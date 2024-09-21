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


#ifndef IPS_CAM__IPS_CAM_NODE_HPP_
#define IPS_CAM__IPS_CAM_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <algorithm>  // For std::find
#include <map>

#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "ips_cam/image_processor.hpp"
#include "ips_cam/usb_cam.hpp"
#include "opencv2/imgproc.hpp"

namespace ips_cam
{

template<typename T>
int findIndex(const std::vector<T> & vec, const T & value)
{
  auto it = std::find(vec.begin(), vec.end(), value);
  if (it != vec.end()) {
    return std::distance(vec.begin(), it);
  } else {
    return -1;      // Return -1 if not found
  }
}

/// @brief generate pose for ROS from image processing tools
/// @param tagPose
/// @param ros_pose
void from_tag_pose(TagPose & tagPose, geometry_msgs::msg::Pose & ros_pose)
{
  ros_pose.position.x = tagPose.x;
  ros_pose.position.y = tagPose.y;
  ros_pose.position.z = tagPose.z;
  // quaternion from angle. Note this is planar motion, with rotation about zhat.
  tf2::Quaternion q;
  tf2::Vector3 z(0.0, 0.0, 1.0);
  q.setRotation(z, tagPose.theta);
  ros_pose.orientation = tf2::toMsg(q);
}


/// @brief An IpsCamNode is a node which contains a camera but
/// provides positions of interesting objects within the field of view. In
/// this sense, it is a sort of an object tracker.
/// Its function in providing camera frames is secondary at best - it
/// is purely for diagnostic purposes. The primary design constraint
/// is accurate positions at a high frame rate.
class IpsCamNode : public rclcpp::Node
{
public:
  explicit IpsCamNode(const rclcpp::NodeOptions & node_options);
  ~IpsCamNode();

  void init();
  void declare_params();
  void get_params();
  void assign_params(const std::vector<rclcpp::Parameter> & parameters);
  void set_v4l2_params();
  bool check_device();
  void update();
  bool take_and_process_image();
  bool load_tracking_yaml();

  // the internal ... camera
  usb_cam::UsbCam * m_camera;


  // the dictionary of pose publishers - one for each object to be tracked
  std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_map;

  // camera parameters
  usb_cam::parameters_t m_parameters;

  // ips files/parameters
  std::string ics_params_path;
  std::string tracking_params_path;
  IcsParams icsParams;
  TrackingParams trackingParams;

  rclcpp::TimerBase::SharedPtr m_timer;

  IndoorCoordSystem ics;

  std::unique_ptr<ObjectTracker> tagFinder;

  cv::Mat detection_image;
};

}  // namespace ips_cam
#endif  // IPS_CAM__IPS_CAM_NODE_HPP_
