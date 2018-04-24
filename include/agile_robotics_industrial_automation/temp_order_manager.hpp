	#pragma once

	#include <list>
	#include <map>
	#include <string>

    #include <geometry_msgs/Pose.h>
	#include <geometry_msgs/PoseArray.h>
	#include <moveit/move_group_interface/move_group_interface.h>
	#include <osrf_gear/LogicalCameraImage.h>
	#include <osrf_gear/Order.h>
	#include <ros/ros.h>
	#include <tf/transform_listener.h>

	#include "agile_robotics_industrial_automation/sensor.hpp"
	#include "agile_robotics_industrial_automation/ur10_controller.hpp"
    typedef std::map<std::string, std::list<std::pair<std::string,geometry_msgs::Pose>>> map_var;
	class OrderManager {
	 public:
	  OrderManager();
	  ~OrderManager();
	  void orderCallback(const osrf_gear::Order& order_msg);
	  void executeOrder();
	  std::string getPartType(std::string object);
	  std::list<map_var> getOrder();

	 private:
	  ros::NodeHandle manager_nh_;
	  ros::Subscriber order_subscriber_;
	  Sensor camera_;
	  UR10Controller robot_;

	  tf::TransformListener part_tf_listener_;
  	 

	  std::string object;
	  map_var order_;
	  std::list<map_var> order_m_;
	  std::map<std::string, std::list<std::string>> scanned_objects_;
	  std::list<geometry_msgs::Pose> drop_pose;
	};