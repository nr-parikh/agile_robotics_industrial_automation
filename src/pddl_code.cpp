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

#include <osrf_gear/AGVControl.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <fstream>
#include <iostream>
#include <typeinfo>
#include "agile_robotics_industrial_automation/order_manager.hpp"

void startCompetition(ros::NodeHandle &node) {
  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success) {
    ROS_ERROR_STREAM(
        "Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

void endCompetition(ros::NodeHandle &node) {
  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/end_competition");

  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;
  start_client.call(srv);
  if (!srv.response.success) {
    ROS_ERROR_STREAM(
        "Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_example_node");

  if (argc == 2) {
    // ROS_ERROR_STREAM(argv[1]);
    std::fstream rFile(argv[1]);

    ros::NodeHandle node;
    OrderManager manager;

    startCompetition(node);

    ros::Duration(2.0).sleep();
    ros::spinOnce();

    auto objects = manager.getOrder();

    int count = 0;
    for (auto it : objects) {
      count += it.second.size();
    }

    std::fstream wFile;
    std::stringstream temp;

    std::string path =
        ros::package::getPath("agile_robotics_industrial_automation");

    wFile.open(path + "/new-group3ariac-problem.pddl", std::ios::out);

    if (rFile.good() and !rFile.eof()) {
      std::string line;
      while (getline(rFile, line)) {
        std::stringstream ss;
        auto found1 = line.find("=(");
        if (found1 != std::string::npos) {
          ss << "\t\t(=(No-of-parts-in-order order) " << count << ")";
          temp << ss.str() << std::endl;
          ss.str(std::string());

          for (auto it : objects) {
            for (auto j = 0; j < it.second.size(); j++) {
              ss << "\t\t(orderlist order " << it.first << "_" << j << ")";
              temp << ss.str() << std::endl;
              ss.str(std::string());
            }
          }
        }
        auto found2 = line.find("orderlist");
        if (found1 == std::string::npos and found2 == std::string::npos) {
          ss << line;
          temp << ss.str() << std::endl;
          ss.str(std::string());
        }
      }

      rFile.close();
      wFile << temp.str();
      wFile.close();

      ros::Duration(0.5).sleep();

      endCompetition(node);

      ROS_WARN_STREAM("Killing the node....");
    }

  } else
    ROS_ERROR_STREAM("No input file given! Quitting...");

  return 0;
}