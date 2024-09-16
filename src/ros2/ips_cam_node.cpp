#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include "ips_cam/ips_cam_node.hpp"
//#include "ips_cam/utils.hpp"

namespace ips_cam
{

IpsCamNode::IpsCamNode(const rclcpp::NodeOptions &node_options)
    : Node("ips_cam", node_options),
      m_camera(new usb_cam::UsbCam()),
      m_parameters(),
      ics(),
      tagFinder(nullptr)
{
    declare_params();
    get_params();
    init();
}

IpsCamNode::~IpsCamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");

  m_timer.reset();

  delete (m_camera);
}

void IpsCamNode::declare_params()
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
    this->declare_parameter("brightness", 50); // 0-255, -1 "leave alone"
    this->declare_parameter("contrast", -1);   // 0-255, -1 "leave alone"
    this->declare_parameter("saturation", -1); // 0-255, -1 "leave alone"
    this->declare_parameter("sharpness", -1);  // 0-255, -1 "leave alone"
    this->declare_parameter("gain", -1);       // 0-100?, -1 "leave alone"
    this->declare_parameter("auto_white_balance", true);
    this->declare_parameter("white_balance", 4000);
    this->declare_parameter("autoexposure", true);
    this->declare_parameter("exposure", 100);
    this->declare_parameter("autofocus", false);
    this->declare_parameter("focus", -1); // 0-255, -1 "leave alone"
    this->declare_parameter("ics_params_file", ""); // setup file
    this->declare_parameter("tracking_params_file", ""); // setup file
}

void IpsCamNode::get_params()
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

void IpsCamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
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
void IpsCamNode::set_v4l2_params()
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
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance_temperature_auto' to %d", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance' to %d", m_parameters.white_balance);
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 0);
    m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
  }

  // check auto exposure
  if (!m_parameters.autoexposure) {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure' to %d", m_parameters.exposure);
    // turn down exposure control (from max of 3)
    m_camera->set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    m_camera->set_v4l_parameter("exposure_absolute", m_parameters.exposure);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 3);
    m_camera->set_v4l_parameter("exposure_auto", 3);
  }

  // check auto focus
  if (m_parameters.autofocus) {
    m_camera->set_auto_focus(1);
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 1);
    m_camera->set_v4l_parameter("focus_auto", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 0);
    m_camera->set_v4l_parameter("focus_auto", 0);
    if (m_parameters.focus >= 0) {
      RCLCPP_INFO(this->get_logger(), "Setting 'focus_absolute' to %d", m_parameters.focus);
      m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
    }
  }
}

void IpsCamNode::init()
{

    if (!check_device()) 
    {
        rclcpp::shutdown();
        return;
    }

    // image processing configs. note that these will
    // throw if they can't find things (like files).
    try
    {       
        icsParams = load_ics_params(ics_params_path);
        trackingParams = load_tracking_params(tracking_params_path);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_ONCE(
            this->get_logger(),
            "Error in tracking setup '%s'", e.what());
        rclcpp::shutdown();
        return;
    }


    if (trackingParams.tag.size() == 0)
    {
        RCLCPP_ERROR_ONCE(
            this->get_logger(),
            "Exiting on config: No tags to track!");
        rclcpp::shutdown();
        return;
    }

    // setup image processing from configs
    ics = EstablishIndoorCoordinateSystem(icsParams);

    tagFinder = std::make_unique<ObjectTracker>(ics, trackingParams.tag_lookup);

    // set the IO method
    usb_cam::io_method_t io_method =
        usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
    if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN)
    {
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

    // iterate through our targets and assign publishers. This is
    // from a nice suggestion by chatgpt 4o

    for (int target : trackingParams.tag)
    {
        std::string topic_name = "object_" + std::to_string(target);
        auto publisher =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, rclcpp::QoS(100));
        publishers_map.emplace(target, publisher);
    }

    detection_image = cv::Mat(m_parameters.image_height, m_parameters.image_width,  CV_8UC1);

    const int period_ms = 1000.0 / m_parameters.framerate;
    m_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&IpsCamNode::update, this));
    RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");

}

bool IpsCamNode::check_device()
{

    // Check if given device name is an available v4l2 device
    auto available_devices = usb_cam::utils::available_devices();
    if (available_devices.find(m_parameters.device_name) == available_devices.end())
    {
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "Device specified is not available or is not a vaild V4L2 device: `" << m_parameters.device_name << "`");
        RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
        for (const auto &device : available_devices)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
            RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
        }
        // rclcpp::shutdown();
        return false;
    }
    return true;
}

void IpsCamNode::update()
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


bool IpsCamNode::take_and_process_image()
{

    usb_cam::buffered_image buff_im = m_camera->get_buffered_image();

    if (buff_im.valid)
    {
        // grab timestamp
        struct timespec timestamp = buff_im.stamp;

        //process the frame
        // form an image suitable for opencv reduction computations.
        cv::Mat src_image(buff_im.height, buff_im.width, CV_8UC2, buff_im.data);
        cv::cvtColor(src_image, detection_image, cv::COLOR_YUV2GRAY_YUYV);

        //done with the buffer - return to v4l2
        m_camera->release_buffered_image(buff_im);

        // process this monochrome image into tag locations
        std::vector<TagPose> tagPoses = tagFinder->Track(detection_image);

        //  convert and publish tag poses
        for (TagPose tagPose : tagPoses)
        {
            // if (hitCount < 1)
            // {
            //   std::cout << "tag pose! " << tagPose.tag << std::endl;
            //   hitCount ++;
            // }
            geometry_msgs::msg::PoseStamped rosPose;
            from_tag_pose(tagPose, rosPose.pose);
            rosPose.header.stamp.sec = timestamp.tv_sec;
            rosPose.header.stamp.nanosec = timestamp.tv_nsec;

            // find the publisher to dispatch this with
            auto it = publishers_map.find(tagPose.tag);
            if (it != publishers_map.end())
            {
                it->second->publish(rosPose);
            }

        }
        return true;

    }
    else
    {
        return false;
    }

}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ips_cam::IpsCamNode)
