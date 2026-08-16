#ifndef AMR_SWEEPER_USB_CAMERAS__MJPEG_HPP_
#define AMR_SWEEPER_USB_CAMERAS__MJPEG_HPP_

extern "C" {
#define __STDC_CONSTANT_MACROS
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libswscale/swscale.h>
}

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace amr_sweeper_usb_cameras
{
namespace formats
{

// Stateful FFmpeg decoder for MJPEG camera frames.
// The decoder keeps FFmpeg contexts alive across frames so the node can avoid
// paying setup costs every time a raw subscriber appears.
class MjpegDecoder
{
public:
  MjpegDecoder();
  ~MjpegDecoder();

  // Decode one MJPEG packet into packed RGB24 output.
  void decode_to_rgb(const char * src, int bytes_used, int width, int height, uint8_t * dest);

private:
  // Ensure the reusable RGB destination frame matches the decoded frame size.
  void ensure_output_frame(int width, int height);
  // Rebuild the scaler only when the decoder reports a new input format or size.
  void ensure_scale_context(AVPixelFormat input_format, bool full_range, int width, int height);

  // FFmpeg decoder implementation for MJPEG packets.
  const AVCodec * m_codec;
  AVCodecContext * m_codec_context;
  // Raw decoded frame produced by libavcodec.
  AVFrame * m_decoded_frame;
  // RGB24 output frame used as the swscale destination.
  AVFrame * m_rgb_frame;
  // Cached pixel-format conversion context.
  SwsContext * m_sws_context;
  // Current RGB frame buffer size in bytes.
  int m_rgb_buffer_size;
  // Cached input format description used to know when the scaler must be rebuilt.
  AVPixelFormat m_last_input_format;
  bool m_last_input_full_range;
  int m_last_width;
  int m_last_height;
};

}  // namespace formats
}  // namespace amr_sweeper_usb_cameras

#endif  // AMR_SWEEPER_USB_CAMERAS__MJPEG_HPP_
