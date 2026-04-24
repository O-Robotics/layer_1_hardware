#ifndef AMR_SWEEPER_USB_CAMERAS__CAMERA_HPP_
#define AMR_SWEEPER_USB_CAMERAS__CAMERA_HPP_

extern "C" {
#include <libavcodec/avcodec.h>
#include <linux/videodev2.h>
}

#include <memory>
#include <string>
#include <vector>

#include "mjpeg.hpp"
#include "utils.hpp"

namespace amr_sweeper_usb_cameras
{

// Runtime configuration for one camera instance.
// These values come directly from the ROS parameter file used with `ros2 run`.
struct CameraParameters
{
  std::string camera_name {"default_camera"};
  std::string device_name {"/dev/video0"};
  std::string frame_id {"camera"};
  std::string camera_info_url {
    "package://amr_sweeper_usb_cameras/config/front_left_camera_info.yaml"};
  int image_width {320};
  int image_height {240};
  int framerate {5};
};

// Image metadata shared between the driver and the ROS publishing layer.
// The capture format stores the negotiated V4L2 settings, while the geometry
// fields describe the decoded RGB output layout.
struct ImageBuffer
{
  size_t width {0};
  size_t height {0};
  size_t bytes_per_line {0};
  size_t size_in_bytes {0};
  v4l2_format capture_format {};
  timespec stamp {};

  void configure(size_t new_width, size_t new_height)
  {
    width = new_width;
    height = new_height;
    bytes_per_line = width * 3;
    size_in_bytes = bytes_per_line * height;
  }
};

// Low-level camera driver for one MJPEG USB camera.
// It owns the V4L2 device handle, capture buffers, and the FFmpeg decoder used
// when a raw RGB image is requested by a subscriber.
class UsbCamera
{
public:
  UsbCamera();
  ~UsbCamera();

  // Open the requested device and negotiate MJPEG capture at the configured size and rate.
  void configure(const CameraParameters & parameters);
  // Queue buffers and start the V4L2 capture stream.
  void start();
  // Stop streaming and release any device resources still held by the driver.
  void shutdown();
  // Capture one frame, always returning the compressed MJPEG payload and optionally
  // decoding the same frame into the provided RGB destination buffer.
  void capture_frame(std::vector<uint8_t> & compressed_destination, char * decoded_destination);

  size_t get_image_width() const {return m_image.width;}
  size_t get_image_height() const {return m_image.height;}
  size_t get_image_size_in_bytes() const {return m_image.size_in_bytes;}
  unsigned int get_image_step() const {return static_cast<unsigned int>(m_image.bytes_per_line);}
  timespec get_image_timestamp() const {return m_image.stamp;}
  const std::string & get_encoding() const {return m_output_encoding;}
  bool is_capturing() const {return m_is_capturing;}

private:
  // Allocate and map the driver-managed streaming buffers used by V4L2 mmap capture.
  void init_mmap();
  // Validate device capabilities and apply the requested MJPEG capture settings.
  void init_device();
  void open_device();
  void close_device();
  void uninit_device();
  // Wait for one frame to become ready and then pull it from the driver.
  void grab_image(std::vector<uint8_t> & compressed_destination, char * decoded_destination);
  // Dequeue one V4L2 buffer, copy out the valid MJPEG payload, and requeue the buffer.
  void read_frame(std::vector<uint8_t> & compressed_destination, char * decoded_destination);
  // Copy the exact compressed payload size and decode only when raw output is needed.
  void process_frame(
    const uint8_t * src,
    size_t bytes_used,
    std::vector<uint8_t> & compressed_destination,
    char * decoded_destination);

  // Path to the selected `/dev/video*` device after symlink resolution.
  std::string m_device_name;
  // Open file descriptor for the V4L2 device.
  int m_fd;
  // Number of mmap buffers requested from the kernel.
  unsigned int m_number_of_buffers;
  // Maximum MJPEG payload size reported by the device for the negotiated format.
  size_t m_capture_buffer_size;
  // mmap buffer metadata for the active capture stream.
  std::vector<utils::Buffer> m_buffers;
  // Shared image metadata used when publishing decoded RGB frames.
  ImageBuffer m_image;
  // FFmpeg-based MJPEG decoder used only for raw subscribers.
  formats::MjpegDecoder m_decoder;
  // Tracks whether the V4L2 stream is currently running.
  bool m_is_capturing;
  // Requested capture frame rate.
  int m_framerate;
  // Offset used to convert monotonic V4L2 timestamps into ROS wall-clock time.
  const time_t m_epoch_time_shift_us;
  // ROS image encoding published on the raw topic.
  std::string m_output_encoding;
};

}  // namespace amr_sweeper_usb_cameras

#endif  // AMR_SWEEPER_USB_CAMERAS__CAMERA_HPP_
