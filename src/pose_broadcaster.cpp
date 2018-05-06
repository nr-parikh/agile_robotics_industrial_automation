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

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <map>
#include "agile_robotics_industrial_automation/sensor_2b.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_node");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<std_msgs::String>("/conveyor/poses", 100);
  ros::Rate loop_rate(10);

  tf::Transform transform;
  tf::Quaternion quat;
  Sensor camera;
  static tf::TransformBroadcaster br;
  // std::map<std::string, geometry_msgs::Pose> temp_map;

  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    auto loop_map = camera.getMap();
    auto vel = camera.getVelocity();
    // ROS_WARN_STREAM("velocity received: " << vel);
    // ROS_WARN_STREAM("TEMP_MAP: " << temp_map.size());
    // ROS_WARN_STREAM("LOOP_MAP: " << loop_map.size());

    if (loop_map.size()) {
      for (auto& i : loop_map) {
        for (auto& j : i.second) {
          transform.setOrigin(
              tf::Vector3(j._pose.position.x, j._pose.position.y, 0.0));
          quat.setRPY(0, 0, 0);
          transform.setRotation(quat);
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                "world", j._frame));
          j._pose.position.y += vel * 0.1;
          ROS_INFO_STREAM("positions: " << j._pose.position.y);
        }
      }
    }
    loop_rate.sleep();
  }

  return 0;
};