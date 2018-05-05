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
  // std::string part = scanned_objects_["gasket_part"].back();

  ROS_INFO_STREAM("scan object"<< scanned_objects_.size());

  osrf_gear::GetMaterialLocations srv;
  ros::ServiceClient get_bin_client_ =  
  manager_nh_.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
 
}

OrderManager::~OrderManager() {}

// void OrderManager::orderCallback(const osrf_gear::Order::ConstPtr& order_msg)
// {
//   for (auto& kit : order_msg->kits) {
//     for (auto& object : kit.objects) {
//       order_[object.type].push_back(scanned_objects_[object.type].front());
//       scanned_objects_[object.type].pop_front();
//     }
//   }
// }

void OrderManager::orderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
  order_ = *order_msg;

}

std::string OrderManager::getPartType(std::string object) {

  if (scanned_objects_[object].size()){

    std::string part = scanned_objects_[object].back();
    scanned_objects_[object].pop_back();
    return part;

   }
   else{

    ROS_ERROR_STREAM("No Parts Left on Bin");
    submitAGV(1,"order_1_kit_0");
    submitAGV(2,"order_0_kit_0");

   } 
}



bool OrderManager::pickAndPlace(const std::pair<std::string,geometry_msgs::Pose> object_prop,int agvnum) {

  ROS_WARN_STREAM("In PicknPlace Function");
  std::string object_type = object_prop.first;
  ROS_INFO_STREAM("Part Type " << object_type);
  std::string object_frame = this->getPartType(object_type);
  auto part_pose = camera_.getPartPose("/world",object_frame);
  geometry_msgs::Quaternion fixed_orientation_;
  
  auto offset=0.011; //dont change
  if (object_type== "gear_part")  // Dont change gear at all now
      offset+= 0.0089;
  else if(object_type== "piston_rod_part") // Works sometimes
      offset+= 0.0052;
  else if(object_type== "disk_part")
      offset+= 0.02;
  else if(object_type== "pulley_part")
      offset+= 0.082;
  else if(object_type== "gasket_part")
      offset+= 0.012;

    part_pose.position.z += offset;

    ROS_INFO_STREAM("Increasing Z for Part"<< object_type << " : " << part_pose.position.z);
    
  bool failed_pick = robot_.pickPart(part_pose);
  ROS_INFO_STREAM("Pick Check =  " << failed_pick);
  ros::Duration(0.1).sleep();
  while(!failed_pick){
        auto part_pose = camera_.getPartPose("/world",object_frame);
      failed_pick = robot_.pickPart(part_pose); 
       ros::spinOnce();
        ros::Duration(0.1).sleep();
  }

 

  geometry_msgs::Pose drop_pose = object_prop.second;
  // if(object_prop.first == "pulley_part")
    drop_pose.position.z += offset; //0.0078

  geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;
  if(agvnum==1){
      StampedPose_in.header.frame_id = "/agv1_load_point_frame";
      StampedPose_in.pose = drop_pose;
      
      part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
      StampedPose_out.pose.position.z += 0.05;
      StampedPose_out.pose.position.y -= 0.2;
    }
  else{
      StampedPose_in.header.frame_id = "/agv2_load_point_frame";
      StampedPose_in.pose = drop_pose;
      
      part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
      StampedPose_out.pose.position.z += 0.05;
      StampedPose_out.pose.position.y += 0.2;
     
      
  }
    auto result = robot_.dropPart(StampedPose_out.pose);

  ROS_WARN_STREAM("End of PicknPlace Function");

  return result;

}

void OrderManager::executeOrder() {
  ROS_INFO_STREAM("Executing order...");
  // geometry_msgs::Pose part_pose;
  bool failed;
  // std::string object_frame;
  std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;
  std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts_higher;

  
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  auto order1_ = order_;
  auto kits1_ = order_.kits;
  
  int finish=0,i=0;
  for (auto kit : order1_.kits) {
    
    for (auto& object : kit.objects) {
         
         ros::spinOnce();
         ros::Duration(0.1).sleep();
         auto kits1_ = order_.kits;

     // For Higher Priority    

                   if(kits1_[i].kit_type != kit.kit_type && finish==0){
                     ROS_WARN_STREAM("HIGH PRIORITY ORDER" );

                     for (auto kitnew : order_.kits) {


                      ROS_INFO_STREAM("Kit Type in High Priority" << kitnew.kit_type);
                      // std::list<std::string> parts = kit.second;
                      // std::string object;
                        for (auto& objectnew : kitnew.objects) {
                          
                          object_prop.first = objectnew.type;
                          object_prop.second = objectnew.pose;
                          int agvnum=1;
                          failed = pickAndPlace(object_prop,agvnum);
                          while (failed) {
                              ROS_WARN_STREAM("Adding the Failed Parts Back into the list of Higher Priority Order");
                              
                              ROS_INFO_STREAM("Adding part : " << object_prop.first);
                              
                              failed = pickAndPlace(object_prop,1);

                          }
                        }
                          ROS_INFO_STREAM("Submitting Kit Type" << kitnew.kit_type);
                          submitAGV(1,kitnew.kit_type);
                          ROS_WARN_STREAM("Submitting AGV 1");
                          int finish=1;

                        }
                           
                     }

      
      object_prop.first = object.type;
      object_prop.second = object.pose;
      int agvnum=2;
      failed = pickAndPlace(object_prop,agvnum);
      while (failed) {
        ROS_WARN_STREAM("Adding the Failed Parts Back into the list of Higher Priority Order");
        // parts.push_front(this->getPartType(kit.first));
        // ROS_WARN_STREAM("Current part list size: " << parts.size());
        // scanned_objects_[object.type].emplace_back(object_frame);
        ROS_WARN_STREAM("Adding part : " << object_prop.first);
        // failed_parts_higher.emplace_back(object_prop);
        failed = pickAndPlace(object_prop,2);
      }
    }
    



      ROS_INFO_STREAM("Parts dropped: " << object);
      ROS_INFO_STREAM("Kit Type" << kit.kit_type);
      submitAGV(2,kit.kit_type);
      ROS_WARN_STREAM("Submitting AGV 2");
      i+=1;
    }
}



void OrderManager::submitAGV(int num, std::string name) {
  // if(num==1){
  std::string s = std::to_string(num);
  ros::ServiceClient start_client =
      manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
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

