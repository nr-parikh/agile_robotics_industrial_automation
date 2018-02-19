#include "agile_robotics_industrial_automation/ur10_controller.hpp"

UR10Controller::UR10Controller() : robot_move_group_("manipulator") {
  // robot_move_group_("manipulator");
  robot_move_group_.setPlanningTime(10);
  robot_move_group_.setNumPlanningAttempts(10);
  robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
  home_position_ = {0.0, 3.0, -1.0, 2.0, 3.5, 4.7, 0};
  sendRobotHome();
}

UR10Controller::~UR10Controller() {}

bool UR10Controller::planner() {
  ROS_INFO_STREAM("Planning started...");
  if (robot_move_group_.plan(robot_planner_) ==
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    plan_success_ = true;
    ROS_INFO_STREAM("Planner succeeded!");
  } else {
    plan_success_ = false;
    ROS_INFO_STREAM("Planner failed!");
  }

  return plan_success_;
}

void UR10Controller::execute() {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  if (this->planner()) {
    robot_move_group_.move();
    ros::Duration(3.0).sleep();
  }
}

void UR10Controller::setTarget(const geometry_msgs::Pose& pose) {
  robot_move_group_.setPoseTarget(pose);
}

void UR10Controller::sendRobotHome() {
  ROS_INFO_STREAM("Going to Home position!");
  ros::Duration(2.0).sleep();
  robot_move_group_.setJointValueTarget(home_position_);
  this->execute();
  ros::Duration(3.0).sleep();
}