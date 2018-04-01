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