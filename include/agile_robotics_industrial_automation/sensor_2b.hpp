// BSD 3-Clause License

// Copyright (c) 2018, Neel Parikh
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <list>
#include <map>
#include <string>
// #include <pair>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>

#include <osrf_gear/LogicalCameraImage.h>

// typedef std::map<std::string, std::pair<int, geometry_msgs::Pose> conv_type; 

class Sensor {
 public:
  Sensor();
  ~Sensor();
  void camera1Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
  void camera2Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
  geometry_msgs::Pose getPartPose(const std::string& src_frame,
                                  const std::string& target_frame);
  std::map<std::string, std::list<std::string>> getParts();
  void scanParts(int cam_number);
  void convertPose(const geometry_msgs::PoseStamped& in);
  geometry_msgs::Pose getConveyorPose();
  std::map<std::string, geometry_msgs::Pose> getMap();

 private:
  ros::NodeHandle sensor_nh_;
  ros::Subscriber camera_1_subscriber_;
  ros::Subscriber camera_2_subscriber_;

  tf::TransformListener camera_tf_listener_;
  tf::StampedTransform camera_tf_transform_;

  osrf_gear::LogicalCameraImage current_parts_1_;
  osrf_gear::LogicalCameraImage current_parts_2_;

  std::map<std::string, std::list<std::string>> parts_list_;

  bool init_, cam_1_, cam_2_;
  int counter_1_;
  int counter_2_, prev_counter_;

  geometry_msgs::PoseStamped conv_pose_;
  std_msgs::Header cam_frame_;
  std::map<std::string, geometry_msgs::Pose> conveyor_parts_;
};
