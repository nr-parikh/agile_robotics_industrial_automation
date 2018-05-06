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

  prev_time_ = ros::Time::now();
}

Sensor::~Sensor() {}

void Sensor::camera1Callback(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg) {
  geometry_msgs::PoseStamped temp;
  temp.header.frame_id = "logical_camera_1_frame";

  curr_time_ = ros::Time::now();
  float delta_t = curr_time_.toSec() - prev_time_.toSec();
  ROS_INFO_STREAM("time difference: " << delta_t);
  if (velocity_ > 0 && image_msg->models.size()) {
    for (auto& i : image_msg->models) {
      if (i.type != "agv1" && i.type != "kit_tray") {
        ROS_INFO_STREAM("temp: " << image_msg->models.back().pose.position.y);
        conv_dist_.emplace_back(image_msg->models.back().pose.position.y);
        break;
      }
    }

    if (conv_dist_.size() == 10) {
      this->findVelocity(delta_t);
    }
  } else if (velocity_ < 0) {
    for (auto& part : conveyor_parts_) {
      for (auto i = part.second.begin(); i != part.second.end(); ++i) {
        i->_pose.position.y += velocity_ * delta_t;
        if (i->_pose.position.y < -3.0) {
          part.second.erase(i);
        }
      }
    }

    for (const auto& part : image_msg->models) {
      if (part.type != "agv1" && part.type != "kit_tray") {
        auto itr = conveyor_parts_.find(part.type);
        temp.pose = part.pose;
        bool flag = false;
        if (itr != conveyor_parts_.end()) {
          for (const auto& i : itr->second) {
            if (this->checkPart(i._pose, part.pose)) {
              flag = true;
              // ROS_WARN_STREAM("breaking loop.");
              break;
            }
          }
        }
        if (!flag) {
          ConveyorPart c;
          c._frame = part.type + "_" + std::to_string(counter_1_);
          c._pose = temp.pose;
          counter_1_++;
          ROS_WARN_STREAM("Part added: " << part.type);
          conveyor_parts_[part.type].emplace_back(c);
        }
      }
    }
  }
  prev_time_ = curr_time_;
}

bool Sensor::checkPart(const geometry_msgs::Pose& point1,
                       const geometry_msgs::Pose& point2) {
  double dist = std::pow(point1.position.x - point2.position.x, 2) +
                std::pow(point1.position.y - point2.position.y, 2) +
                std::pow(point1.position.z - point2.position.z, 2);
  double angle = std::pow(point1.orientation.x - point2.orientation.x, 2) +
                 std::pow(point1.orientation.y - point2.orientation.y, 2) +
                 std::pow(point1.orientation.z - point2.orientation.z, 2) +
                 std::pow(point1.orientation.w - point2.orientation.w, 2);

  return dist < std::pow(0.1, 2) && angle < std::pow(0.1, 2);
}

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

std::map<std::string, std::vector<Sensor::ConveyorPart>> Sensor::getMap() {
  return this->conveyor_parts_;
}

void Sensor::findVelocity(const double& d_t) {
  double* vel = new double[conv_dist_.size()];
  std::adjacent_difference(conv_dist_.begin(), conv_dist_.end(), vel,
                           [d_t](const double& first, const double& second) {
                             return (first - second) / d_t;
                           });

  double total = 0;
  for (auto i = 1; i < conv_dist_.size(); ++i) {
    total += vel[i];
  }
  delete[] vel;
  velocity_ = total / double(conv_dist_.size() - 1);
  ROS_WARN_STREAM("velocity is : " << total);
}

double Sensor::getVelocity() { return velocity_; }