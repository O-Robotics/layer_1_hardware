#ifndef AMR_SWEEPER_USB_CAMERAS__UTILS_HPP_
#define AMR_SWEEPER_USB_CAMERAS__UTILS_HPP_

extern "C" {
#include <fcntl.h>
}

#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include <linux/videodev2.h>

namespace fs = std::filesystem;

namespace amr_sweeper_usb_cameras
{
namespace utils
{

// Metadata for one mmap capture buffer owned by the V4L2 driver.
struct Buffer
{
  char * start {nullptr};
  size_t length {0};
};

// Convert V4L2 monotonic timestamps into epoch-based timestamps that ROS messages use.
inline time_t get_epoch_time_shift_us()
{
  timeval epoch_time {};
  timespec monotonic_time {};

  gettimeofday(&epoch_time, nullptr);
  clock_gettime(CLOCK_MONOTONIC, &monotonic_time);

  const int64_t uptime_us =
    monotonic_time.tv_sec * 1000000 + static_cast<int64_t>(std::round(monotonic_time.tv_nsec / 1000.0));
  const int64_t epoch_us = epoch_time.tv_sec * 1000000 + epoch_time.tv_usec;

  return static_cast<time_t>(epoch_us - uptime_us);
}

// Translate one V4L2 buffer timestamp into a ROS-friendly `timespec`.
inline timespec calc_img_timestamp(const timeval & buffer_time, const time_t & epoch_time_shift_us)
{
  timespec image_timestamp {};
  int64_t buffer_time_us = (buffer_time.tv_sec * 1000000) + buffer_time.tv_usec + epoch_time_shift_us;
  image_timestamp.tv_sec = buffer_time_us / 1000000;
  image_timestamp.tv_nsec = (buffer_time_us % 1000000) * 1000;
  return image_timestamp;
}

// Retry `ioctl` on EINTR so transient signal interruptions do not break capture.
inline int xioctl(int fd, uint64_t request, void * arg)
{
  int result = 0;
  do {
    result = ioctl(fd, request, arg);
  } while (-1 == result && EINTR == errno);
  return result;
}

// Enumerate accessible V4L2 devices so invalid or stale device paths fail early.
inline std::map<std::string, v4l2_capability> available_devices()
{
  std::map<std::string, v4l2_capability> devices;
  const std::string v4l2_symlinks_dir = "/sys/class/video4linux/";

  for (const auto & device_symlink : fs::directory_iterator(v4l2_symlinks_dir)) {
    if (!fs::is_symlink(device_symlink)) {
      continue;
    }

    const std::string device_path = fs::canonical(
      v4l2_symlinks_dir + fs::read_symlink(device_symlink).generic_string()).string();

    std::ifstream uevent_file(device_path + "/uevent");
    std::string line;
    std::string device_name;
    while (std::getline(uevent_file, line)) {
      const auto dev_name_index = line.find("DEVNAME=");
      if (dev_name_index != std::string::npos) {
        device_name = "/dev/" + line.substr(dev_name_index + 8);
        break;
      }
    }

    if (device_name.empty()) {
      continue;
    }

    const int fd = open(device_name.c_str(), O_RDONLY);
    if (fd == -1) {
      continue;
    }

    v4l2_capability device_capabilities {};
    if (xioctl(fd, VIDIOC_QUERYCAP, &device_capabilities) == 0) {
      devices[device_name] = device_capabilities;
    }
    close(fd);
  }

  return devices;
}

}  // namespace utils
}  // namespace amr_sweeper_usb_cameras

#endif  // AMR_SWEEPER_USB_CAMERAS__UTILS_HPP_
