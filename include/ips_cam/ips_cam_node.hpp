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

#include "ips_cam/cam_node.hpp"

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
  // to meters!
  ros_pose.position.x = tagPose.x * 1e-3;
  ros_pose.position.y = tagPose.y * 1e-3;
  ros_pose.position.z = tagPose.z * 1e-3;
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
class IpsCamNode : public CamNode
{
public:
  explicit IpsCamNode(const rclcpp::NodeOptions & node_options);
  virtual ~IpsCamNode();

  void init() override;
  void update() override;
  bool take_and_process_image() override;
  bool load_tracking_yaml();

  // the dictionary of pose publishers - one for each object to be tracked
  std::map<int, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_map;

  // ips files/parameters
  TrackingParams trackingParams;

  IndoorCoordSystem ics;

  std::unique_ptr<ObjectTracker> tagFinder;

  cv::Mat detection_image;
};

}  // namespace ips_cam
#endif  // IPS_CAM__IPS_CAM_NODE_HPP_
