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
#include "ips_cam/ips_cam_node.hpp"

namespace ips_cam
{

IpsCamNode::IpsCamNode(const rclcpp::NodeOptions & node_options)
: CamNode(node_options),
  ics(),
  tagFinder(nullptr)
{
  // init is virtual, so wait until after construction
}

IpsCamNode::~IpsCamNode()
{
  // the base class destructor should handle everything
}


void IpsCamNode::init()
{
  if (!check_device()) {
    rclcpp::shutdown();
    return;
  }

  // image processing configs. note that these will
  // throw if they can't find things (like files).
  try {
    icsParams = load_ics_params(ics_params_path);
    trackingParams = load_tracking_params(tracking_params_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(),
      "Error in tracking setup '%s'", e.what());
    rclcpp::shutdown();
    return;
  }

  if (trackingParams.tag.size() == 0) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(),
      "Exiting on config: No tags to track!");
    rclcpp::shutdown();
    return;
  }

  // setup image processing from configs
  ics = EstablishIndoorCoordinateSystem(icsParams);

  // prepare for different-sized image stream
  ics.ScaleIntrinsics(m_parameters.image_width, m_parameters.image_height);

  tagFinder = std::make_unique<ObjectTracker>(ics, trackingParams.tag_lookup);

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

  // iterate through our targets and assign publishers. This is
  // from a nice suggestion by chatgpt 4o

  for (int target : trackingParams.tag) {
    std::string topic_name = "object_" + std::to_string(target);
    auto publisher =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, rclcpp::QoS(100));
    publishers_map.emplace(target, publisher);
  }

  detection_image = cv::Mat(m_parameters.image_height, m_parameters.image_width, CV_8UC1);

  const int period_ms = 1000.0 / m_parameters.framerate;
  m_timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
    std::bind(&IpsCamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
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

  if (buff_im.valid) {
    // grab timestamp
    struct timespec timestamp = buff_im.stamp;

    // process the frame
    // form an image suitable for opencv reduction computations.
    cv::Mat src_image(buff_im.height, buff_im.width, CV_8UC2, buff_im.data);
    cv::cvtColor(src_image, detection_image, cv::COLOR_YUV2GRAY_YUYV);

    // done with the buffer - return to v4l2
    m_camera->release_buffered_image(buff_im);

    // process this monochrome image into tag locations
    std::vector<TagPose> tagPoses = tagFinder->Track(detection_image);

    //  convert and publish tag poses
    for (TagPose tagPose : tagPoses) {
      geometry_msgs::msg::PoseStamped rosPose;
      from_tag_pose(tagPose, rosPose.pose);
      rosPose.header.stamp.sec = timestamp.tv_sec;
      rosPose.header.stamp.nanosec = timestamp.tv_nsec;

      // note that for visualization in rviz2, a static transform is handy
      // e.g., ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
      rosPose.header.frame_id = "/world";

      // find the publisher to dispatch this with
      auto it = publishers_map.find(tagPose.tag);
      if (it != publishers_map.end()) {
        it->second->publish(rosPose);
      }
    }
    return true;
  } else {
    return false;
  }
}

}  // namespace ips_cam
