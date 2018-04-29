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

#include "agile_robotics_industrial_automation/sensor_2b.hpp"

Sensor::Sensor() {
  camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 1,
                                              &Sensor::camera1Callback, this);
  camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 1,
                                              &Sensor::camera2Callback, this);
  counter_1_ = 1;
  counter_2_ = 1;

  init_ = false;
  cam_1_ = false;
  cam_2_ = false;
}

Sensor::~Sensor() {}

void Sensor::camera1Callback(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  geometry_msgs::PoseStamped temp;
  temp.header.frame_id = "logical_camera_1_frame";

  for (auto& i : image_msg->models) {
    if (i.type == "gasket_part") {
      temp.pose = i.pose;
      this->convertPose(temp);
      if (conveyor_parts_.find("gasket_part_" + std::to_string(counter_1_)) ==
          conveyor_parts_.end()) {
        ROS_INFO_STREAM("inside callback if ......");
        conveyor_parts_["gasket_part_" + std::to_string(counter_1_)] =
            this->getConveyorPose();
        counter_1_++;
      }
    }
  }
}

// void Sensor::camera1Callback(
//     const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
//   geometry_msgs::PoseStamped temp;
//   temp.header.frame_id = "logical_camera_1_frame";
//   for (auto& i : image_msg->models) {
//     if (i.type == "gasket_part") {
//       temp.pose = i.pose;
//       this->convertPose(temp);
//       for (auto it = conveyor_parts_.begin(); it != conveyor_parts_.end();
//            it++) {
//         ROS_INFO_STREAM("here I am...");
//         auto t = it->first.find("gasket_part");
//         ROS_INFO_STREAM("found value");
//         if (t == std::string::npos) {
//           conveyor_parts_["gasket_part_" + std::to_string(counter_1_)] =
//               this->getConveyorPose();
//           counter_1_++;
//           // ROS_INFO_STREAM("inside callback if.......");
//         }
//       }
//     }
//   }
// }

void Sensor::camera2Callback(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  current_parts_2_ = *image_msg;
  if (!init_) {
    this->scanParts(2);
    init_ = true;
  }
}

void Sensor::scanParts(const int cam_number) {
  for (auto& msg : current_parts_2_.models) {
    std::string part = "logical_camera_2_" + msg.type + "_" +
                       std::to_string(counter_2_) + "_frame";
    parts_list_[msg.type].emplace_back(part);
    counter_2_++;
  }
}

geometry_msgs::Pose Sensor::getPartPose(const std::string& src_frame,
                                        const std::string& target_frame) {
  geometry_msgs::Pose part_pose;

  ROS_INFO_STREAM("Getting part pose...");

  camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                       ros::Duration(20));
  camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                      camera_tf_transform_);

  part_pose.position.x = camera_tf_transform_.getOrigin().x();
  part_pose.position.y = camera_tf_transform_.getOrigin().y();
  part_pose.position.z = camera_tf_transform_.getOrigin().z();

  return part_pose;
}

void Sensor::convertPose(const geometry_msgs::PoseStamped& in) {
  camera_tf_listener_.transformPose("/world", in, conv_pose_);
}

std::map<std::string, std::list<std::string>> Sensor::getParts() {
  return parts_list_;
}

geometry_msgs::Pose Sensor::getConveyorPose() { return this->conv_pose_.pose; }

std::map<std::string, geometry_msgs::Pose> Sensor::getMap() {
  return this->conveyor_parts_;
}