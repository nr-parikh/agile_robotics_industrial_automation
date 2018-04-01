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

#include "agile_robotics_industrial_automation/sensor.hpp"

Sensor::Sensor() {
  camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10,
                                              &Sensor::camera1Callback, this);
  camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 10,
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
  if (init_) {
    return;
  }
  current_parts_1_ = *image_msg;
  this->scanParts(1);
}

void Sensor::camera2Callback(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  if (init_) {
    return;
  }
  current_parts_2_ = *image_msg;
  this->scanParts(2);
}

void Sensor::scanParts(const int cam_number) {
  if (cam_number == 1) {
    for (auto& msg : current_parts_1_.models) {
      std::string part = "logical_camera_1_" + msg.type + "_" +
                         std::to_string(counter_1_) + "_frame";
      parts_list_[msg.type].push_back(part);
      counter_1_++;
      cam_1_ = true;
    }
  } else {
    for (auto& msg : current_parts_2_.models) {
      std::string part = "logical_camera_2_" + msg.type + "_" +
                         std::to_string(counter_2_) + "_frame";
      parts_list_[msg.type].push_back(part);
      counter_2_++;
      cam_2_ = true;
    }
  }

  if (cam_1_ && cam_2_) {
    init_ = true;
  }
}

geometry_msgs::Pose Sensor::getPartPose(const std::string& src_frame,
                                        const std::string& target_frame) {
  geometry_msgs::Pose part_pose;

  ROS_INFO_STREAM("Getting part pose...");

  if (init_) {
    camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                         ros::Duration(20));
    camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                        camera_tf_transform_);

    part_pose.position.x = camera_tf_transform_.getOrigin().x();
    part_pose.position.y = camera_tf_transform_.getOrigin().y();
    part_pose.position.z = camera_tf_transform_.getOrigin().z();

  } else {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    this->scanParts(1);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    this->scanParts(2);

    part_pose = this->getPartPose(src_frame, target_frame);
  }

  return part_pose;
}

std::map<std::string, std::list<std::string>> Sensor::getParts() {
  return parts_list_;
}