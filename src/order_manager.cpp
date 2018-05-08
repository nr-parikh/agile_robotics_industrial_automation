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
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

OrderManager::OrderManager() {
  order_subscriber_ = manager_nh_.subscribe("/ariac/orders", 10,
                                            &OrderManager::orderCallback, this);
  scanned_objects_ = camera_.getParts();

  conv_subscriber_ = manager_nh_.subscribe("/conveyor_parts", 10,
                                           &OrderManager::convCallback, this);
  dummy_subscriber_ = manager_nh_.subscribe("/ariac/competition_state", 10,
                                            &OrderManager::dummyCallback, this);
  prev_time_ = ros::Time::now();

  ROS_INFO_STREAM("scan object" << scanned_objects_.size());
}

OrderManager::~OrderManager() {}

void OrderManager::orderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
  order_ = *order_msg;
}

void OrderManager::convCallback(
    const osrf_gear::LogicalCameraImage::ConstPtr& msg) {
  parts_on_conv_ = *msg;
}

void OrderManager::dummyCallback(const std_msgs::String& dummy_msg) {
  // ROS_ERROR_STREAM("inside callback...");
  if (ros::Time::now().toSec() - prev_time_.toSec() > 200) {
    ROS_WARN_STREAM("Kit name 1: " << error_kit_list_.first._kit_name);
    ROS_WARN_STREAM("Kit name 2: " << error_kit_list_.second._kit_name);
    this->submitAGV(error_kit_list_.first._agv_num,
                    error_kit_list_.first._kit_name);
    this->submitAGV(error_kit_list_.second._agv_num,
                    error_kit_list_.second._kit_name);
  }
}

std::string OrderManager::getPartType(std::string object) {
  // auto itr = parts_on_conv_.models.find(object);
  std::string part;
  int count1,count2;  

  if (object == "piston_rod_part" || "gasket_part" || "gear_part") {
    int count1 = 0;
    while (!found_|| count1 < 5) {
      count1++;
      found_ = false;
      ros::spinOnce();
      // ROS_INFO_STREAM("parts_on_conv_ size: " <<
      // parts_on_conv_.models.size());
      if (parts_on_conv_.models.size()) {
        // ROS_ERROR_STREAM("parts type" << parts_on_conv_.models[0].type);
        // ROS_ERROR_STREAM("parts size" << parts_on_conv_.models.size());

        // ROS_ERROR_STREAM("parts size" << parts_on_conv_.models.find(object));
        for (const auto& itr : parts_on_conv_.models) {
          // ROS_INFO_STREAM("parts_on_conv_: " << itr.type);
          if (itr.type == object) {
            if (itr.pose.position.y > 0 && itr.pose.position.y < 2.0) {
              // part = itr->second.back()._frame;
              part = "conveyor";
              // conv_parts_[object].pop_back();
              ROS_INFO_STREAM("Part on conveyor...");
              found_ = true;
              break;
            }
          }
        }
      }
      if (!found_) {
        if (scanned_objects_[object].size()) {
          part = scanned_objects_[object].back();
          scanned_objects_[object].pop_back();
          ROS_INFO_STREAM("Part on bin...");
          found_ = true;
          break;
        }
      }
    }
  }

  // priotizing bin over conveyor
  else {
    int count2 = 0;
    while (!found_ || count2 < 5) {
      count2++;
      found_ = false;
      if (scanned_objects_[object].size()) {
        part = scanned_objects_[object].back();
        scanned_objects_[object].pop_back();
        ROS_INFO_STREAM("Part on bin...");
        found_ = true;
        break;
      }
    }
    if (!found_) {
      ros::spinOnce();
      // ROS_INFO_STREAM("parts_on_conv_ size: " <<
      // parts_on_conv_.models.size());
      if (parts_on_conv_.models.size()) {
        for (const auto& itr : parts_on_conv_.models) {
          // ROS_INFO_STREAM("parts_on_conv_: " << itr.type);
          if (itr.type == object) {
            if (itr.pose.position.y > 0 && itr.pose.position.y < 2.0) {
              // part = itr->second.back()._frame;
              part = "conveyor";
              // conv_parts_[object].pop_back();
              ROS_INFO_STREAM("Part on conveyor...");
              found_ = true;
              break;
            }
          }
        }
      }
    }
  }
  found_ = false;
  // if (count1 >= 4 || count2 >= 4){
  //   error = "invalid";  
  // }
  
  return part;
}

bool OrderManager::pickAndPlace(
    const std::pair<std::string, geometry_msgs::Pose> object_prop, int agvnum) {
  ROS_WARN_STREAM("In PicknPlace Function");
  std::string object_type = object_prop.first;
  ROS_INFO_STREAM("Part Type " << object_type);
  std::string object_frame = this->getPartType(object_type);
  geometry_msgs::Pose part_pose;
  if (object_frame == "conveyor") {
    ros::spinOnce();
    if (parts_on_conv_.models.size()) {
      for (const auto& itr : parts_on_conv_.models) {
        if (itr.type == object_type) {
          if (itr.pose.position.y > 0 && itr.pose.position.y < -0.2) {
            part_pose = itr.pose;
            ROS_INFO_STREAM("part_pose: " << part_pose);
          }
        }
      }
    }
  } else {
    part_pose = camera_.getPartPose("/world", object_frame);
  }

  geometry_msgs::Quaternion fixed_orientation_;

  float offset = 0.011;            // dont change
  if (object_type == "gear_part")  // Dont change gear at all now
    offset += 0.0089;
  else if (object_type == "piston_rod_part")  // Works sometimes
    offset += 0.0052;
  else if (object_type == "disk_part")
    offset += 0.02;
  else if (object_type == "pulley_part")
    offset += 0.0845;
  else if (object_type == "gasket_part")
    offset += 0.012;

  part_pose.position.z += offset;
  double diffroll, diffpitch;
  geometry_msgs::Pose drop_pose = object_prop.second;

  bool failed_pick;
  if (object_frame == "conveyor") {
    failed_pick = robot_.goToConveyor(part_pose, object_type);
  } else {
    if (object_type == "pulley_part") {
      tf::Quaternion q;
      tf::Matrix3x3 m;
      double r_1, p_1, y_1, y_2, r_2, p_2;
      q = {part_pose.orientation.x, part_pose.orientation.y,
           part_pose.orientation.z, part_pose.orientation.w};
      m.setRotation(q);
      m.getRPY(r_1, p_1, y_1);
      q = {drop_pose.orientation.x, drop_pose.orientation.y,
           drop_pose.orientation.z, drop_pose.orientation.w};
      m.setRotation(q);
      m.getRPY(r_2, p_2, y_2);

      ROS_ERROR_STREAM("Part POSE : roll" << r_1 << "pitch" << p_1 << "yaw"
                                          << y_1);
      ROS_ERROR_STREAM("DROP POSE : roll" << r_2 << "pitch" << p_2 << "yaw"
                                          << y_2);
      diffroll = abs(r_1 - r_2);
      diffpitch = abs(p_1 - p_2);
      if (diffroll > 1.74 || diffpitch > 1.74) {
        bool failed_pick = robot_.flipPart(part_pose);
        while (!failed_pick) {
          ros::Duration(1).sleep();
          part_pose = camera_.getPartPose("/world", object_frame);
          // part_pose.position.z += 0.011;
          failed_pick = robot_.pickPartpulley(part_pose);
          ros::spinOnce();
          ros::Duration(0.1).sleep();
          ROS_INFO_STREAM("pulley set");
        }
      } else {
        ROS_ERROR_STREAM("NOT FLIPPING PULLEY");
        bool failed_pick = robot_.pickPartpulley(part_pose);
        while (!failed_pick) {
          ros::Duration(1).sleep();
          part_pose = camera_.getPartPose("/world", object_frame);
          // part_pose.position.z += 0.011;
          failed_pick = robot_.pickPartpulley(part_pose);
          ros::spinOnce();
          ros::Duration(0.1).sleep();
          ROS_INFO_STREAM("pulley set");
        }
      }
    }

    else {
      bool failed_pick = robot_.pickPart(part_pose);
      ROS_INFO_STREAM("Pick Check =  " << failed_pick);
      ros::Duration(0.1).sleep();
      while (!failed_pick) {
        auto part_pose = camera_.getPartPose("/world", object_frame);
        failed_pick = robot_.pickPart(part_pose);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
    }
  }

  // geometry_msgs::Pose drop_pose = object_prop.second;
  if (object_prop.first == "pulley_part") drop_pose.position.z += offset;

  geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
  if (agvnum == 1) {
    StampedPose_in.header.frame_id = "/agv1_load_point_frame";
    StampedPose_in.pose = drop_pose;

    part_tf_listener_.transformPose("/world", StampedPose_in, StampedPose_out);
    StampedPose_out.pose.position.z += 0.035;
    StampedPose_out.pose.position.y -= 0.15;
  } else {
    StampedPose_in.header.frame_id = "/agv2_load_point_frame";
    StampedPose_in.pose = drop_pose;

    part_tf_listener_.transformPose("/world", StampedPose_in, StampedPose_out);
    StampedPose_out.pose.position.z += 0.035;
    StampedPose_out.pose.position.y += 0.15;
  }
  auto result = robot_.dropPart(StampedPose_out.pose);

  ROS_WARN_STREAM("End of PicknPlace Function");

  // bool result = true;
  return result;
}

void OrderManager::executeOrder() {
  auto prev_time_ = ros::Time::now();
  ROS_INFO_STREAM("Executing order...");
  // geometry_msgs::Pose part_pose;
  bool failed;
  // std::string object_frame;
  std::list<std::pair<std::string, geometry_msgs::Pose>> failed_parts;
  std::list<std::pair<std::string, geometry_msgs::Pose>> failed_parts_higher;

  ros::spinOnce();
  ros::Duration(1.0).sleep();
  auto order1_ = order_;
  auto kits1_ = order_.kits;

  int finish = 0, i = 0;
  
  for (auto kit : order1_.kits) {
    for (auto& object : kit.objects) {
      error_kit_list_.first._agv_num = 2;
      error_kit_list_.first._kit_name = kit.kit_type;

      ros::spinOnce();
      ros::Duration(0.1).sleep();
      auto kits1_ = order_.kits;

      // For Higher Priority
      if (kits1_[i].kit_type != kit.kit_type && finish == 0) {
        ROS_WARN_STREAM("HIGH PRIORITY ORDER");

        for (auto kitnew : order_.kits) {
          ROS_INFO_STREAM("Kit Type in High Priority" << kitnew.kit_type);
          error_kit_list_.first._agv_num = 1;
          error_kit_list_.first._kit_name = kitnew.kit_type;
          for (auto& objectnew : kitnew.objects) {
            object_prop.first = objectnew.type;
            object_prop.second = objectnew.pose;
            int agvnum = 1;
            failed = pickAndPlace(object_prop, agvnum);
            while (failed) {
              ROS_WARN_STREAM(
                  "Adding the Failed Parts Back into the list of Higher "
                  "Priority Order");

              ROS_INFO_STREAM("Adding part : " << object_prop.first);

              failed = pickAndPlace(object_prop, 1);
            }
          }
          ROS_INFO_STREAM("Submitting Kit Type" << kitnew.kit_type);
          submitAGV(1, kitnew.kit_type);
          ROS_WARN_STREAM("Submitting AGV 1");
          int finish = 1;
        }
      }

      object_prop.first = object.type;
      object_prop.second = object.pose;
      int agvnum = 2;
      failed = pickAndPlace(object_prop, agvnum);
      while (failed) {
        ROS_WARN_STREAM(
            "Adding the Failed Parts Back into the list of Higher Priority "
            "Order");
        ROS_WARN_STREAM("Adding part : " << object_prop.first);
        failed = pickAndPlace(object_prop, 2);
      }
    }

    ROS_INFO_STREAM("Parts dropped: " << object);
    ROS_INFO_STREAM("Kit Type" << kit.kit_type);
    submitAGV(2, kit.kit_type);
    ROS_WARN_STREAM("Submitting AGV 2");
    i += 1;
  }
  	  ros::spinOnce();
      ros::Duration(0.1).sleep();
      kits1_ = order_.kits;

       if (kits1_[i].kit_type == "order_1_kit_0" ) {
        ROS_WARN_STREAM("HIGH PRIORITY ORDER");

        for (auto kitnew : order_.kits) {
          ROS_INFO_STREAM("Kit Type in High Priority" << kitnew.kit_type);
          error_kit_list_.first._agv_num = 1;
          error_kit_list_.first._kit_name = kitnew.kit_type;
          for (auto& objectnew : kitnew.objects) {
            object_prop.first = objectnew.type;
            object_prop.second = objectnew.pose;
            int agvnum = 1;
            failed = pickAndPlace(object_prop, agvnum);
            while (failed) {
              ROS_WARN_STREAM(
                  "Adding the Failed Parts Back into the list of Higher "
                  "Priority Order");

              ROS_INFO_STREAM("Adding part : " << object_prop.first);

              failed = pickAndPlace(object_prop, 1);
            }
          }
          ROS_INFO_STREAM("Submitting Kit Type" << kitnew.kit_type);
          submitAGV(1, kitnew.kit_type);
          ROS_WARN_STREAM("Submitting AGV 1");
          int finish = 1;
        }
      }	


}

void OrderManager::submitAGV(int num, std::string name) {
  // if(num==1){
  std::string s = std::to_string(num);
  ros::ServiceClient start_client =
      manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv" + s);
  //
  osrf_gear::AGVControl srv;
  srv.request.kit_type = name;

  if (!start_client.exists()) {
    ROS_INFO("Waiting for the client to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Service started.");
  }

  start_client.call(srv);

  if (!srv.response.success) {
    ROS_ERROR_STREAM("Service failed!");
  } else
    ROS_INFO("Service succeeded.");
}
