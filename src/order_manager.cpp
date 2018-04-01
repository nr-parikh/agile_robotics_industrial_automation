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

#include "agile_robotics_industrial_automation/order_manager.hpp"

OrderManager::OrderManager() {
  order_subscriber_ = manager_nh_.subscribe("/ariac/orders", 10,
                                            &OrderManager::orderCallback, this);
  scanned_objects_ = camera_.getParts();
}

OrderManager::~OrderManager() {}

void OrderManager::orderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
  for (auto& kit : order_msg->kits) {
    for (auto& object : kit.objects) {
      order_[object.type].push_back(scanned_objects_[object.type].front());
      scanned_objects_[object.type].pop_front();
      // ROS_WARN_STREAM("object.type:>>>>>>>>>>" << object.type <<
      // "typeid>>>>>>"
      //                                          <<
      //                                          typeid(object.type).name());
    }
  }
}

std::string OrderManager::getPartType(std::string object) {
  // ROS_WARN_STREAM("<<<<<<<>>>>>>>>>>"
  //                 << object << ">>>>>>>>>>type:" << typeid(object).name());
  std::string part = scanned_objects_[object].back();
  scanned_objects_[object].pop_back();
  return part;
}

void OrderManager::executeOrder() {
  ROS_INFO_STREAM("Executing order...");
  geometry_msgs::Pose part_pose;
  bool success;

  ros::spinOnce();
  ros::Duration(3.0).sleep();

  for (auto& kit : order_) {
    ROS_INFO_STREAM(">>>>>>>>" << kit.first);
    std::list<std::string> parts = kit.second;
    std::string object;
    while (!parts.empty()) {
      object = this->getPartType(kit.first);
      part_pose = camera_.getPartPose("/world", object);
      part_pose.position.z = part_pose.position.z + 0.075;
      robot_.pickPart(part_pose);
      parts.pop_front();
      ROS_INFO_STREAM("Part picked: " << object);
      success = robot_.dropPart(part_pose);
      if (!success) {
        ROS_WARN_STREAM("Part lost and cannot drop!!!");
        parts.push_front(this->getPartType(kit.first));
        ROS_WARN_STREAM("Current part list size: " << parts.size());
      }
    }

    ROS_INFO_STREAM("Part dropped: " << object);
  }
}

std::map<std::string, std::list<std::string>> OrderManager::getOrder() {
  return this->order_;
}
