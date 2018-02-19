#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <osrf_gear/VacuumGripperControl.h>

class UR10Controller {
 public:
  UR10Controller();
  ~UR10Controller();
  bool planner();
  void execute();
  void setTarget(const geometry_msgs::Pose& target);
  void sendRobotHome();
  void dropPart();
  void gripperToggle(const bool& state);
  void pickPart(geometry_msgs::Pose& part_pose);

 private:
  ros::NodeHandle ur10_nh_;
  ros::ServiceClient gripper_client_;

  tf::TransformListener robot_tf_listener_;
  tf::StampedTransform robot_tf_transform_;
  tf::TransformListener agv_tf_listener_;
  tf::StampedTransform agv_tf_transform_;

  geometry_msgs::Pose target_pose_;

  moveit::planning_interface::MoveGroupInterface robot_move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

  osrf_gear::VacuumGripperControl gripper_service_;

  bool plan_success_;
  std::vector<double> home_position_;
  geometry_msgs::Quaternion fixed_orientation_;
  std::vector<double> end_position_;
  double offset_;
  geometry_msgs::Pose agv_position_;
};