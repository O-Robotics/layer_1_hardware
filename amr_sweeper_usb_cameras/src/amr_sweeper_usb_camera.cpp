#define CLEAR(x) memset(&(x), 0, sizeof(x))

extern "C" {
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
}

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "amr_sweeper_usb_camera.hpp"

namespace amr_sweeper_usb_cameras
{
namespace formats
{

namespace
{

AVPixelFormat normalize_input_format(AVPixelFormat input_format)
{
  switch (input_format) {
    case AV_PIX_FMT_YUVJ420P:
      return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ422P:
      return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ444P:
      return AV_PIX_FMT_YUV444P;
    case AV_PIX_FMT_YUVJ440P:
      return AV_PIX_FMT_YUV440P;
    case AV_PIX_FMT_YUVJ411P:
      return AV_PIX_FMT_YUV411P;
    default:
      return input_format;
  }
}

bool is_full_range_input(const AVFrame * frame)
{
  if (frame->color_range == AVCOL_RANGE_JPEG) {
    return true;
  }

  switch (static_cast<AVPixelFormat>(frame->format)) {
    case AV_PIX_FMT_YUVJ420P:
    case AV_PIX_FMT_YUVJ422P:
    case AV_PIX_FMT_YUVJ444P:
    case AV_PIX_FMT_YUVJ440P:
    case AV_PIX_FMT_YUVJ411P:
      return true;
    default:
      return false;
  }
}

}  // namespace

// Build the FFmpeg decoder once so it can be reused for every requested raw frame.
MjpegDecoder::MjpegDecoder()
: m_codec(avcodec_find_decoder(AV_CODEC_ID_MJPEG)),
  m_codec_context(nullptr),
  m_decoded_frame(av_frame_alloc()),
  m_rgb_frame(av_frame_alloc()),
  m_sws_context(nullptr),
  m_rgb_buffer_size(0),
  m_last_input_format(AV_PIX_FMT_NONE),
  m_last_input_full_range(false),
  m_last_width(0),
  m_last_height(0)
{
  if (m_codec == nullptr) {
    throw std::runtime_error("MJPEG decoder is not available");
  }

  m_codec_context = avcodec_alloc_context3(m_codec);
  if (m_codec_context == nullptr) {
    throw std::runtime_error("Failed to allocate codec context");
  }

  // Swscale emits recurring warnings for JPEG full-range input and missing
  // optimized conversion kernels. Keep FFmpeg logs focused on actual errors.
  av_log_set_level(AV_LOG_ERROR);

  if (avcodec_open2(m_codec_context, m_codec, nullptr) < 0) {
    throw std::runtime_error("Failed to open MJPEG decoder");
  }
}

// Release all FFmpeg-owned objects in the reverse order they depend on each other.
MjpegDecoder::~MjpegDecoder()
{
  if (m_sws_context != nullptr) {
    sws_freeContext(m_sws_context);
  }
  if (m_rgb_frame != nullptr) {
    av_frame_free(&m_rgb_frame);
  }
  if (m_decoded_frame != nullptr) {
    av_frame_free(&m_decoded_frame);
  }
  if (m_codec_context != nullptr) {
    avcodec_free_context(&m_codec_context);
  }
}

// Reallocate the RGB output frame only when the decoded image size changes.
void MjpegDecoder::ensure_output_frame(int width, int height)
{
  if (m_rgb_frame->width == width && m_rgb_frame->height == height && m_rgb_buffer_size != 0) {
    return;
  }

  av_frame_unref(m_rgb_frame);
  m_rgb_frame->format = AV_PIX_FMT_RGB24;
  m_rgb_frame->width = width;
  m_rgb_frame->height = height;

  if (av_frame_get_buffer(m_rgb_frame, 32) < 0) {
    throw std::runtime_error("Failed to allocate RGB frame buffer");
  }

  m_rgb_buffer_size = av_image_get_buffer_size(AV_PIX_FMT_RGB24, width, height, 1);
  if (m_rgb_buffer_size < 0) {
    throw std::runtime_error("Failed to calculate RGB frame size");
  }
}

// Rebuild the conversion pipeline only when FFmpeg reports a different source format.
void MjpegDecoder::ensure_scale_context(
  AVPixelFormat input_format,
  bool full_range,
  int width,
  int height)
{
  const auto normalized_format = normalize_input_format(input_format);

  if (m_sws_context != nullptr &&
    m_last_input_format == normalized_format &&
    m_last_input_full_range == full_range &&
    m_last_width == width &&
    m_last_height == height)
  {
    return;
  }

  if (m_sws_context != nullptr) {
    sws_freeContext(m_sws_context);
  }

  m_sws_context = sws_getContext(
    width, height, normalized_format,
    width, height, AV_PIX_FMT_RGB24,
    SWS_FAST_BILINEAR,
    nullptr, nullptr, nullptr);

  if (m_sws_context == nullptr) {
    throw std::runtime_error("Failed to create scaling context");
  }

  const auto * coefficients = sws_getCoefficients(SWS_CS_DEFAULT);
  if (sws_setColorspaceDetails(
      m_sws_context,
      coefficients,
      full_range ? 1 : 0,
      coefficients,
      0,
      0,
      1 << 16,
      1 << 16) < 0)
  {
    throw std::runtime_error("Failed to configure color range conversion");
  }

  m_last_input_format = normalized_format;
  m_last_input_full_range = full_range;
  m_last_width = width;
  m_last_height = height;
}

// Decode a single MJPEG payload and convert it to packed RGB24.
void MjpegDecoder::decode_to_rgb(
  const char * src,
  int bytes_used,
  int width,
  int height,
  uint8_t * dest)
{
  AVPacket * packet = av_packet_alloc();
  if (packet == nullptr) {
    throw std::runtime_error("Failed to allocate packet");
  }

  if (av_new_packet(packet, bytes_used) < 0) {
    av_packet_free(&packet);
    throw std::runtime_error("Failed to allocate MJPEG packet");
  }

  memcpy(packet->data, src, bytes_used);

  if (avcodec_send_packet(m_codec_context, packet) < 0) {
    av_packet_free(&packet);
    throw std::runtime_error("Failed to submit MJPEG packet");
  }
  av_packet_free(&packet);

  const int result = avcodec_receive_frame(m_codec_context, m_decoded_frame);
  if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
    return;
  }
  if (result < 0) {
    throw std::runtime_error("Failed to decode MJPEG frame");
  }

  const int decoded_width = m_decoded_frame->width > 0 ? m_decoded_frame->width : width;
  const int decoded_height = m_decoded_frame->height > 0 ? m_decoded_frame->height : height;
  ensure_output_frame(decoded_width, decoded_height);
  ensure_scale_context(
    static_cast<AVPixelFormat>(m_decoded_frame->format),
    is_full_range_input(m_decoded_frame),
    decoded_width,
    decoded_height);

  sws_scale(
    m_sws_context,
    m_decoded_frame->data,
    m_decoded_frame->linesize,
    0,
    decoded_height,
    m_rgb_frame->data,
    m_rgb_frame->linesize);

  av_image_copy_to_buffer(
    dest,
    m_rgb_buffer_size,
    m_rgb_frame->data,
    m_rgb_frame->linesize,
    AV_PIX_FMT_RGB24,
    decoded_width,
    decoded_height,
    1);
}

}  // namespace formats

// Initialize the driver in a fully stopped state. Actual device work begins in `configure()`.
UsbCamera::UsbCamera()
: m_device_name(),
  m_fd(-1),
  m_number_of_buffers(2),
  m_capture_buffer_size(0),
  m_buffers(m_number_of_buffers),
  m_image(),
  m_decoder(),
  m_is_capturing(false),
  m_framerate(0.0),
  m_epoch_time_shift_us(utils::get_epoch_time_shift_us()),
  m_output_encoding("rgb8")
{
}

UsbCamera::~UsbCamera()
{
  shutdown();
}

// Copy out the exact compressed payload bytes and optionally decode the same
// frame into a caller-provided RGB buffer.
void UsbCamera::process_frame(
  const uint8_t * src,
  size_t bytes_used,
  std::vector<uint8_t> & compressed_destination,
  char * decoded_destination)
{
  compressed_destination.resize(bytes_used);
  if (bytes_used > 0) {
    std::memcpy(compressed_destination.data(), src, bytes_used);
  }

  if (decoded_destination != nullptr) {
    m_decoder.decode_to_rgb(
      reinterpret_cast<const char *>(src),
      static_cast<int>(bytes_used),
      static_cast<int>(m_image.width),
      static_cast<int>(m_image.height),
      reinterpret_cast<uint8_t *>(decoded_destination));
  }
}

// Pull one frame from the kernel, update its timestamp, then return the buffer
// to the driver immediately so streaming can continue.
bool UsbCamera::read_frame(
  std::vector<uint8_t> & compressed_destination,
  char * decoded_destination)
{
  v4l2_buffer buf {};
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  if (-1 == utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
    if (errno == EAGAIN) {
      return false;
    }
    throw std::runtime_error("Unable to retrieve frame with mmap");
  }

  m_image.stamp = utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);
  process_frame(
    reinterpret_cast<const uint8_t *>(m_buffers[buf.index].start),
    static_cast<size_t>(buf.bytesused),
    compressed_destination,
    decoded_destination);

  if (-1 == utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
    throw std::runtime_error("Unable to exchange buffer with the driver");
  }

  return true;
}

// Stop the V4L2 stream and release any mapped buffers and file handles.
void UsbCamera::shutdown()
{
  if (m_fd == -1) {
    return;
  }

  if (m_is_capturing) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == utils::xioctl(m_fd, VIDIOC_STREAMOFF, &type)) {
      throw std::runtime_error("Unable to stop capture stream");
    }
    m_is_capturing = false;
  }

  uninit_device();
  close_device();
  m_number_of_buffers = 2;
  m_buffers.assign(m_number_of_buffers, {});
}

// Unmap all buffers that were previously provided by the V4L2 driver.
void UsbCamera::uninit_device()
{
  for (auto & buffer : m_buffers) {
    if (buffer.start == nullptr) {
      continue;
    }

    munmap(buffer.start, buffer.length);
    buffer.start = nullptr;
    buffer.length = 0;
  }
}

// Ask V4L2 for a set of mmap buffers and map each one into this process.
void UsbCamera::init_mmap()
{
  v4l2_requestbuffers req {};
  CLEAR(req);
  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == utils::xioctl(m_fd, static_cast<int>(VIDIOC_REQBUFS), &req)) {
    if (errno == EINVAL) {
      throw std::runtime_error("Device does not support memory mapping");
    }
    throw std::runtime_error("Unable to initialize memory mapping");
  }

  if (req.count < m_number_of_buffers) {
    throw std::runtime_error("Device did not provide enough capture buffers");
  }

  m_number_of_buffers = req.count;
  m_buffers.assign(m_number_of_buffers, {});

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    v4l2_buffer buf {};
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = current_buffer;

    if (-1 == utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYBUF), &buf)) {
      throw std::runtime_error("Unable to query buffer metadata");
    }

    m_buffers[current_buffer].length = buf.length;
    m_buffers[current_buffer].start = reinterpret_cast<char *>(
      mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, buf.m.offset));

    if (m_buffers[current_buffer].start == MAP_FAILED) {
      throw std::runtime_error("Unable to map capture buffer");
    }
  }
}

// Verify that the selected device can capture streaming video, then request
// MJPEG at the configured size and frame rate.
void UsbCamera::init_device()
{
  v4l2_capability cap {};

  if (-1 == utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCAP), &cap)) {
    throw std::runtime_error("Unable to query device capabilities");
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    throw std::runtime_error("Device is not a video capture device");
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    throw std::runtime_error("Device does not support streaming I/O");
  }

  m_image.capture_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_image.capture_format.fmt.pix.width = m_image.width;
  m_image.capture_format.fmt.pix.height = m_image.height;
  m_image.capture_format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  m_image.capture_format.fmt.pix.field = V4L2_FIELD_ANY;

  if (-1 == utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_FMT), &m_image.capture_format)) {
    throw std::runtime_error("Unable to set MJPEG capture format");
  }

  m_image.configure(
    m_image.capture_format.fmt.pix.width,
    m_image.capture_format.fmt.pix.height);
  m_capture_buffer_size = m_image.capture_format.fmt.pix.sizeimage;
  if (m_capture_buffer_size == 0) {
    throw std::runtime_error("Device returned an invalid MJPEG payload size");
  }

  v4l2_streamparm stream_params {};
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_PARM), &stream_params) < 0) {
    throw std::runtime_error("Unable to read capture stream settings");
  }

  const auto requested_fps = static_cast<uint32_t>(std::lround(m_framerate));
  if (requested_fps == 0) {
    throw std::runtime_error("Frame rate must be greater than zero");
  }

  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = requested_fps;
  if (utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0) {
    throw std::runtime_error("Unable to set camera frame rate");
  }

  if (stream_params.parm.capture.timeperframe.numerator != 0 &&
    stream_params.parm.capture.timeperframe.denominator != 0)
  {
    m_framerate =
      static_cast<double>(stream_params.parm.capture.timeperframe.denominator) /
      static_cast<double>(stream_params.parm.capture.timeperframe.numerator);
  }

  init_mmap();
}

void UsbCamera::close_device()
{
  if (m_fd == -1) {
    return;
  }
  if (-1 == close(m_fd)) {
    throw std::runtime_error("Failed to close camera device");
  }
  m_fd = -1;
}

// Open the configured camera device path in nonblocking read/write mode.
void UsbCamera::open_device()
{
  struct stat st {};

  if (-1 == stat(m_device_name.c_str(), &st)) {
    throw std::runtime_error("Unable to stat camera device");
  }
  if (!S_ISCHR(st.st_mode)) {
    throw std::runtime_error("Camera device path is not a character device");
  }

  m_fd = open(m_device_name.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (m_fd == -1) {
    throw std::runtime_error("Unable to open camera device");
  }
}

// Store the requested camera settings and prepare the device for streaming.
void UsbCamera::configure(const CameraParameters & parameters)
{
  m_device_name = parameters.device_name;
  m_framerate = parameters.framerate;
  m_image.configure(parameters.image_width, parameters.image_height);

  open_device();
  init_device();
}

// Queue every capture buffer once and then switch the driver stream on.
void UsbCamera::start()
{
  if (m_is_capturing) {
    return;
  }

  for (unsigned int i = 0; i < m_number_of_buffers; ++i) {
    v4l2_buffer buf {};
    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (-1 == utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
      throw std::runtime_error("Unable to queue capture buffer");
    }
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
    throw std::runtime_error("Unable to start capture stream");
  }

  m_is_capturing = true;
}

// Capture one frame. The compressed payload is always returned; raw decode is optional.
void UsbCamera::capture_frame(std::vector<uint8_t> & compressed_destination, char * decoded_destination)
{
  if (m_image.width == 0 || m_image.height == 0) {
    compressed_destination.clear();
    return;
  }

  compressed_destination.clear();
  grab_image(compressed_destination, decoded_destination);
}

// Wait up to five seconds for the next frame to become available.
void UsbCamera::grab_image(std::vector<uint8_t> & compressed_destination, char * decoded_destination)
{
  fd_set fds;
  timeval tv {};

  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  const int ready = select(m_fd + 1, &fds, nullptr, nullptr, &tv);
  if (ready == -1) {
    if (errno == EINTR) {
      return;
    }
    throw std::runtime_error("select() failed while waiting for a camera frame");
  }
  if (ready == 0) {
    throw std::runtime_error("Camera frame wait timed out");
  }

  // Drain the ready queue so we always return the freshest frame instead of
  // publishing a buffer that may have been waiting behind older frames.
  while (read_frame(compressed_destination, decoded_destination)) {
  }
}

}  // namespace amr_sweeper_usb_cameras
