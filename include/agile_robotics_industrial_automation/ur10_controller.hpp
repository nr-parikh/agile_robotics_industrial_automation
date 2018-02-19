#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2/transform_listener.h>

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

 private:
  ros::NodeHandle ur10_nh_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform tf_transform_;

  geometry_msgs::Pose target_pose_;

  moveit::planning_interface::MoveGroupInterface robot_move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

  bool plan_success_;
  std::vector<double> home_position_ ;
};