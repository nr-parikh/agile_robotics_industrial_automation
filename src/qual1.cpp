#include <osrf_gear/AGVControl.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
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

void submitAGV(ros::NodeHandle &node) {
  ros::ServiceClient start_client =
      node.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");

  if (!start_client.exists()) {
    ROS_INFO("Waiting for the client to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Service started.");
  }

  osrf_gear::AGVControl srv;
  srv.request.kit_type = "order_0_kit_0";
  start_client.call(srv);

  if (!srv.response.success) {
    ROS_ERROR_STREAM("Service failed!");
  } else
    ROS_INFO("Service succeeded.");
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

  ros::NodeHandle node;

  OrderManager manager;

  startCompetition(node);

  ros::Duration(2.0).sleep();

  manager.executeOrder();

  ros::Duration(0.5).sleep();

  submitAGV(node);

  ros::Duration(1.0).sleep();

  endCompetition(node);

  ROS_WARN_STREAM("Killing the node....");

  return 0;
}