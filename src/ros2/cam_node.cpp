// Copyright 2021 Evan Flynn
// Copyright 2014 Robert Bosch, LLC
// Copyright 2024 Stuart Johnson
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Evan Flynn nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include "ips_cam/cam_node.hpp"

namespace ips_cam
{


CamNode::CamNode(const rclcpp::NodeOptions & node_options)
: Node("ips_cam", node_options),
  m_camera(new usb_cam::UsbCam()),
  do_snapshot_(false),
  m_parameters()
{
  declare_params();
  get_params();
}


CamNode::~CamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");

  if (m_timer != nullptr) {
    m_timer.reset();
  }

  if (m_camera != nullptr) {
    m_camera->shutdown();
    delete (m_camera);
  }
}

void CamNode::declare_params()
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("av_device_format", "YUV422P");
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("brightness", 50);    // 0-255, -1 "leave alone"
  this->declare_parameter("contrast", -1);      // 0-255, -1 "leave alone"
  this->declare_parameter("saturation", -1);    // 0-255, -1 "leave alone"
  this->declare_parameter("sharpness", -1);     // 0-255, -1 "leave alone"
  this->declare_parameter("gain", -1);          // 0-100?, -1 "leave alone"
  this->declare_parameter("auto_white_balance", true);
  this->declare_parameter("white_balance", 4000);
  this->declare_parameter("autoexposure", true);
  this->declare_parameter("exposure", 100);
  this->declare_parameter("autofocus", false);
  this->declare_parameter("focus", -1);    // 0-255, -1 "leave alone"
  this->declare_parameter("ics_params_file", "");    // setup file
  this->declare_parameter("tracking_params_file", "");    // setup file
}

void CamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {
      "camera_name", "camera_info_url", "frame_id", "framerate", "image_height", "image_width",
      "io_method", "pixel_format", "av_device_format", "video_device", "brightness", "contrast",
      "saturation", "sharpness", "gain", "auto_white_balance", "white_balance", "autoexposure",
      "exposure", "autofocus", "focus", "ics_params_file", "tracking_params_file"
    }
  );

  assign_params(parameters);
}

std::string resolve_device_path(const std::string & path)
{
  if (std::filesystem::is_symlink(path)) {
    // For some reason read_symlink only returns videox
    return "/dev/" + std::string(std::filesystem::read_symlink(path));
  }
  return path;
}

void CamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
{
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      m_parameters.camera_name = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      m_parameters.camera_info_url = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      m_parameters.frame_id = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      m_parameters.framerate = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      m_parameters.image_height = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      m_parameters.image_width = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      m_parameters.io_method_name = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      m_parameters.pixel_format_name = parameter.value_to_string();
    } else if (parameter.get_name() == "av_device_format") {
      m_parameters.av_device_format = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      m_parameters.device_name = resolve_device_path(parameter.value_to_string());
    } else if (parameter.get_name() == "brightness") {
      m_parameters.brightness = parameter.as_int();
    } else if (parameter.get_name() == "contrast") {
      m_parameters.contrast = parameter.as_int();
    } else if (parameter.get_name() == "saturation") {
      m_parameters.saturation = parameter.as_int();
    } else if (parameter.get_name() == "sharpness") {
      m_parameters.sharpness = parameter.as_int();
    } else if (parameter.get_name() == "gain") {
      m_parameters.gain = parameter.as_int();
    } else if (parameter.get_name() == "auto_white_balance") {
      m_parameters.auto_white_balance = parameter.as_bool();
    } else if (parameter.get_name() == "white_balance") {
      m_parameters.white_balance = parameter.as_int();
    } else if (parameter.get_name() == "autoexposure") {
      m_parameters.autoexposure = parameter.as_bool();
    } else if (parameter.get_name() == "exposure") {
      m_parameters.exposure = parameter.as_int();
    } else if (parameter.get_name() == "autofocus") {
      m_parameters.autofocus = parameter.as_bool();
    } else if (parameter.get_name() == "focus") {
      m_parameters.focus = parameter.as_int();
      // non-camera params
    } else if (parameter.get_name() == "ics_params_file") {
      ics_params_path = parameter.value_to_string();
    } else if (parameter.get_name() == "tracking_params_file") {
      tracking_params_path = parameter.value_to_string();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

/// @brief Send current parameters to V4L2 device
/// TODO(flynneva): should this actuaully be part of UsbCam class?
void CamNode::set_v4l2_params()
{
  // set camera parameters
  if (m_parameters.brightness >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'brightness' to %d", m_parameters.brightness);
    m_camera->set_v4l_parameter("brightness", m_parameters.brightness);
  }

  if (m_parameters.contrast >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'contrast' to %d", m_parameters.contrast);
    m_camera->set_v4l_parameter("contrast", m_parameters.contrast);
  }

  if (m_parameters.saturation >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'saturation' to %d", m_parameters.saturation);
    m_camera->set_v4l_parameter("saturation", m_parameters.saturation);
  }

  if (m_parameters.sharpness >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'sharpness' to %d", m_parameters.sharpness);
    m_camera->set_v4l_parameter("sharpness", m_parameters.sharpness);
  }

  if (m_parameters.gain >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'gain' to %d", m_parameters.gain);
    m_camera->set_v4l_parameter("gain", m_parameters.gain);
  }

  // check auto white balance
  if (m_parameters.auto_white_balance) {
    m_camera->set_v4l_parameter("white_balance_automatic", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance_automatic' to %d", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance' to %d", m_parameters.white_balance);
    m_camera->set_v4l_parameter("white_balance_automatic", 0);
    m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
  }

  // check auto exposure
  if (!m_parameters.autoexposure) {
    RCLCPP_INFO(this->get_logger(), "Setting 'auto_exposure' to %d", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure' to %d", m_parameters.exposure);
    // turn down exposure control (from max of 3)
    m_camera->set_v4l_parameter("auto_exposure", 1);
    // change the exposure level
    m_camera->set_v4l_parameter("exposure_time_absolute", m_parameters.exposure);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 3);
    m_camera->set_v4l_parameter("auto_exposure", 3);
  }

  // check auto focus
  if (m_parameters.autofocus) {
    m_camera->set_auto_focus(1);
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 1);
    m_camera->set_v4l_parameter("focus_automatic_continuous", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 0);
    m_camera->set_v4l_parameter("focus_automatic_continuous", 0);
    if (m_parameters.focus >= 0) {
      RCLCPP_INFO(this->get_logger(), "Setting 'focus_absolute' to %d", m_parameters.focus);
      m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
    }
  }
}

void CamNode::init()
{
  if (!check_device()) {
    rclcpp::shutdown();
    return;
  }

  // image processing configs. note that these will
  // throw if they can't find things (like files).
  try {
    icsParams = load_ics_params(ics_params_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(),
      "Error in tracking setup '%s'", e.what());
    rclcpp::shutdown();
    return;
  }

  // set the IO method
  usb_cam::io_method_t io_method =
    usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
  if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(),
      "Unknown IO method '%s'", m_parameters.io_method_name.c_str());
    rclcpp::shutdown();
    return;
  }

  // configure the camera
  m_camera->configure(m_parameters, io_method);

  set_v4l2_params();

  // start the camera
  m_camera->start();

  // start the snapshot service
  using std::placeholders::_1; using std::placeholders::_2;
  srv_ = this->create_service<ips_cam::srv::Snapshot>(
    "/ips_cam/snapshot",
    std::bind(&CamNode::snapshot_callback, this, _1, _2)
  );

  const int period_ms = 1000.0 / m_parameters.framerate;
  m_timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
    std::bind(&CamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
}

bool CamNode::check_device()
{
  // Check if given device name is an available v4l2 device
  auto available_devices = usb_cam::utils::available_devices();
  if (available_devices.find(m_parameters.device_name) == available_devices.end()) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "Device specified is not available or is not a valid V4L2 device: `"
        << m_parameters.device_name << "`");
    RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
    for (const auto & device : available_devices) {
      RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
      RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
    }
    // rclcpp::shutdown();
    return false;
  }
  return true;
}

void CamNode::update()
{
  if (m_camera->is_capturing()) {
    // If the camera exposure longer higher than the framerate period
    // then that caps the framerate.
    // auto t0 = now();
    bool isSuccessful = take_and_process_image();
    if (!isSuccessful) {
      RCLCPP_WARN_ONCE(this->get_logger(), "USB camera did not respond in time.");
    }
  }
}

void CamNode::snapshot_callback(
  const std::shared_ptr<ips_cam::srv::Snapshot::Request> req,
  const std::shared_ptr<ips_cam::srv::Snapshot::Response> res
)
{
  snapshot_filename_ = req->filename;
  res->accepted = true;
  res->message = "image write set for : " + snapshot_filename_;
  do_snapshot_ = true;
}

bool CamNode::take_and_process_image()
{

  usb_cam::buffered_image buff_im = m_camera->get_buffered_image();

  bool ok = false;

  if (buff_im.valid) {
    // whether or not you are using this frame, you gotta take it and flush it.
    // otherwise, you get the first frame the camera took on startup. Sigh.
    if (do_snapshot_) {
      // process the frame
      // form an image suitable for opencv reduction computations.
      cv::Mat src_image(buff_im.height, buff_im.width, CV_8UC2, buff_im.data);
      cv::Mat snapshot_image;
      cv::cvtColor(src_image, snapshot_image, cv::COLOR_YUV2BGR_YUYV);

      std::ostringstream oss;
      oss << "snapshot: " << buff_im.height << " ; " << buff_im.width;
      std::string oss_string = oss.str();

      // done with the buffer - return to v4l2
      m_camera->release_buffered_image(buff_im);

      do_snapshot_ = false;

      RCLCPP_INFO(this->get_logger(), oss_string.c_str());

      // write the image to a file
      ok = cv::imwrite(snapshot_filename_, snapshot_image);
    } else {
      // done with the buffer - return to v4l2
      m_camera->release_buffered_image(buff_im);
      ok = true;
    }

    return ok;
  } else {
    return false;
  }
}

}  // namespace ips_cam
