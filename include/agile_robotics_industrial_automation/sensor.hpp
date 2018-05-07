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

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Model.h>

class Sensor {
 public:
  struct ConveyorPart {
    std::string _frame;
    geometry_msgs::Pose _pose;
    size_t _count;
  };
  Sensor();
  ~Sensor();
  void camera1Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
  void camera2Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
  void camera3Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
  void camera4Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
  void camera5Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);

  geometry_msgs::Pose getPartPose(const std::string& src_frame,
                                  const std::string& target_frame);
  std::map<std::string, std::list<std::string>> getParts();
  void scanParts(int cam_number);
  void convertPose(const geometry_msgs::PoseStamped& in);
  geometry_msgs::PoseStamped getPartPose(
      const geometry_msgs::PoseStamped& input, std::string reference);
  geometry_msgs::Pose getConveyorPose();
  std::map<std::string, std::vector<ConveyorPart>> getMap();
  bool checkPart(const geometry_msgs::Pose& point1,
                 const geometry_msgs::Pose& point2, double tolerance=0.1);

  void findVelocity(const double& d_t);
  double getVelocity();
  void update(osrf_gear::LogicalCameraImage& msg, double delta_t);

 private:
  ros::NodeHandle sensor_nh_;
  ros::Subscriber camera_1_subscriber_;
  ros::Subscriber camera_2_subscriber_;
  ros::Subscriber camera_3_subscriber_;
  ros::Subscriber camera_4_subscriber_;
  ros::Subscriber camera_5_subscriber_;
  ros::Publisher conv_pub_;

  tf::TransformListener camera_tf_listener_;
  tf::StampedTransform camera_tf_transform_;

  osrf_gear::LogicalCameraImage current_parts_1_;
  osrf_gear::LogicalCameraImage current_parts_2_;
  osrf_gear::LogicalCameraImage current_parts_3_;
  osrf_gear::LogicalCameraImage current_parts_4_;
  osrf_gear::Model part_;

  std::map<std::string, std::list<std::string>> parts_list_;

  bool init_, cam_1_, cam_2_, cam_3_, cam_4_;
  bool init_1_, init_2_, init_3_, init_4_;
  int counter_1_;
  int counter_2_;
  int counter_3_;
  int counter_4_;
  int counter_5_;

  geometry_msgs::PoseStamped conv_pose_;
  std_msgs::Header cam_frame_;
  std::map<std::string, std::vector<ConveyorPart>> conveyor_parts_;

  ros::Time curr_time_, prev_time_;
  std::vector<double> conv_dist_;
  double velocity_ = 1.0;  
};
