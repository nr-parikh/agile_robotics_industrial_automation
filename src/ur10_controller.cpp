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

#include "agile_robotics_industrial_automation/ur10_controller.hpp"

UR10Controller::UR10Controller() : robot_move_group_("manipulator") {
  robot_move_group_.setPlanningTime(100);
  robot_move_group_.setNumPlanningAttempts(10);
  robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
  robot_move_group_.setMaxVelocityScalingFactor(0.9);
  robot_move_group_.setMaxAccelerationScalingFactor(0.9);
  // robot_move_group_.setEndEffector("moveit_ee");
  robot_move_group_.allowReplanning(true);
  home_joint_pose_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
  offset_ = 0.025;

  gripper_subscriber_ = gripper_nh_.subscribe(
      "/ariac/gripper/state", 10, &UR10Controller::gripperCallback, this);

  robot_tf_listener_.waitForTransform("linear_arm_actuator", "ee_link",
                                      ros::Time(0), ros::Duration(10));
  robot_tf_listener_.lookupTransform("/linear_arm_actuator", "/ee_link",
                                     ros::Time(0), robot_tf_transform_);

  sendRobot(home_joint_pose_);

  fixed_orientation_.x = robot_tf_transform_.getRotation().x();
  fixed_orientation_.y = robot_tf_transform_.getRotation().y();
  fixed_orientation_.z = robot_tf_transform_.getRotation().z();
  fixed_orientation_.w = robot_tf_transform_.getRotation().w();
  
  // tf::quaternionMsgToTF(fixed_orientation_,q);
  // tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


  end_position_ = home_joint_pose_;
  end_position_[0] = -2.2;
  end_position_[1] = 4.5;
  end_position_[2] = -1.2;

  robot_tf_listener_.waitForTransform("world", "ee_link", ros::Time(0),
                                      ros::Duration(10));
  robot_tf_listener_.lookupTransform("/world", "/ee_link", ros::Time(0),
                                     robot_tf_transform_);

  home_cart_pose_.position.x = robot_tf_transform_.getOrigin().x();
  home_cart_pose_.position.y = robot_tf_transform_.getOrigin().y();
  home_cart_pose_.position.z = robot_tf_transform_.getOrigin().z();
  home_cart_pose_.orientation.x = robot_tf_transform_.getRotation().x();
  home_cart_pose_.orientation.y = robot_tf_transform_.getRotation().y();
  home_cart_pose_.orientation.z = robot_tf_transform_.getRotation().z();
  home_cart_pose_.orientation.w = robot_tf_transform_.getRotation().w();

  agv_tf_listener_.waitForTransform("world", "agv2_load_point_frame",
                                    ros::Time(0), ros::Duration(10));
  agv_tf_listener_.lookupTransform("/world", "/agv2_load_point_frame",
                                   ros::Time(0), agv_tf_transform_);
  agv_position_.position.x = agv_tf_transform_.getOrigin().x();
  agv_position_.position.y = agv_tf_transform_.getOrigin().y();
  agv_position_.position.z = agv_tf_transform_.getOrigin().z() + 4 * offset_;

  gripper_client_ = ur10_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
      "/ariac/gripper/control");
  counter_ = 0;
  drop_flag_ = false;
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
    ROS_WARN_STREAM("Planner failed!");
  }

  return plan_success_;
}

// void UR10Controller::execute() {
//   ros::AsyncSpinner spinner(4);
//   spinner.start();
//   // ros::Duration(1.0).sleep();
//   // robot_move_group_.move();
//   // robot_move_group_.setMaxVelocityScalingFactor(0.5);
//   // robot_move_group_.setMaxAccelerationScalingFactor(0.5);
//   robot_move_group_.execute(robot_planner_);
//   // ros::Duration(2.0).sleep();
// }

void UR10Controller::execute() {
  ros::AsyncSpinner spinner(4);
  spinner.start();
  if (this->planner()) {
    robot_move_group_.move();
    ros::Duration(0.5).sleep();
  }
}

void UR10Controller::goToTarget(const geometry_msgs::Pose& pose) {
  target_pose_.orientation = fixed_orientation_;
  target_pose_.position = pose.position;
  ros::AsyncSpinner spinner(4);
  // target_pose_.position.z += 0.025;
  robot_move_group_.setPoseTarget(target_pose_);
  // this->execute();
  spinner.start();
  if (this->planner()) {
    robot_move_group_.move();
    ros::Duration(0.5).sleep();
  }
  ROS_INFO_STREAM("Point reached...");
}

void UR10Controller::goToTarget(
    std::initializer_list<geometry_msgs::Pose> list) {
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::vector<geometry_msgs::Pose> waypoints;
  for (auto i : list) {

    tf::Quaternion q, final;
    tf::Matrix3x3 m;
    double r_h, p_h, y_h, y_t;
    q = {i.orientation.x, i.orientation.y, i.orientation.z,
         i.orientation.w};
    m.setRotation(q);
    m.getRPY(r_h, p_h, y_t);
    q = {fixed_orientation_.x, fixed_orientation_.y,fixed_orientation_.z,
         fixed_orientation_.w};
    m.setRotation(q);
    m.getRPY(r_h, p_h, y_h);
    final = tf::createQuaternionFromRPY(r_h, p_h, y_t);
    i.orientation.x = final.getX();
    i.orientation.y = final.getY();
    i.orientation.z = final.getZ();
    i.orientation.w = final.getW();

    // i.orientation.x = fixed_orientation_.x;
    // i.orientation.y = fixed_orientation_.y;
    // i.orientation.z = fixed_orientation_.z;
    // i.orientation.w = fixed_orientation_.w;
    waypoints.emplace_back(i);
  }

  moveit_msgs::RobotTrajectory traj;
  auto value =
      robot_move_group_.computeCartesianPath(waypoints, 0.001, 0.0, traj, true);

  ROS_WARN_STREAM("VALUE: " << value);

  robot_planner_.trajectory_ = traj;

  if (value >= 0.3) {
    robot_move_group_.execute(robot_planner_);
    ros::Duration(2.0).sleep();
  } else {
    ROS_ERROR_STREAM("Safe Trajectory not found!");
    home_joint_pose_ = {0.0, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
    sendRobot(home_joint_pose_);
    auto value =
      robot_move_group_.computeCartesianPath(waypoints, 0.001, 0.0, traj, true);
    robot_planner_.trajectory_ = traj;
    robot_move_group_.execute(robot_planner_);
    ros::Duration(2.0).sleep();

  }
}

void UR10Controller::sendRobot(std::vector<double> pose) {
  // ros::Duration(2.0).sleep();
  robot_move_group_.setJointValueTarget(pose);
  // this->execute();
  ros::AsyncSpinner spinner(4);
  spinner.start();
  if (this->planner()) {
    robot_move_group_.move();
    ros::Duration(0.5).sleep();
  }

  // ros::Duration(2.0).sleep();
}

void UR10Controller::gripperToggle(const bool& state) {
  gripper_service_.request.enable = state;
  gripper_client_.call(gripper_service_);
  ros::Duration(0.5).sleep();
  // if (gripper_client_.call(gripper_service_)) {
  if (gripper_service_.response.success) {
    ROS_INFO_STREAM("Gripper activated!");
  } else {
    ROS_WARN_STREAM("Gripper activation failed!");
  }
}

// bool UR10Controller::dropPart(geometry_msgs::Pose part_pose) {
//   counter_++;

//   pick = false;
//   drop = true;

//   ROS_WARN_STREAM("Dropping the part number: " << counter_);

//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   // this->gripper_state_check(part_pose);

//   if (drop == false) {
//     // ROS_INFO_STREAM("I am stuck here..." << object);
//     ros::Duration(2.0).sleep();
//     return drop;
//   }
//   ROS_INFO_STREAM("Dropping on AGV...");

//   // agv_position_.position.x -= 0.1;
//   // if (counter_ == 1) {
//   //   agv_position_.position.y -= 0.1;
//   // }
//   // if (counter_ >= 2) {
//   //   agv_position_.position.y += 0.1;
//   //   // agv_position_.position.x +=0.1;
//   // }

//   auto temp_pose = part_pose;
//   // auto temp_pose = agv_position_;
//   temp_pose.position.z += 0.35;
//   // temp_pose.position.y += 0.5;

//   // this->setTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   this->goToTarget({temp_pose, part_pose});
//   ros::Duration(1).sleep();
//   ROS_INFO_STREAM("Actuating the gripper...");
//   this->gripperToggle(false);

//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(end_position_);
//   // this->execute();
//   // ros::Duration(1.0).sleep();

//   ROS_INFO_STREAM("Going to home...");
//   // this->sendRobotHome();
//   // temp_pose = home_cart_pose_;
//   // temp_pose.position.z -= 0.05;
//   this->goToTarget({temp_pose, home_cart_pose_});
//   return drop;
// }

bool UR10Controller::dropPart(geometry_msgs::Pose part_pose) {
  // counter_++;

  
  drop_flag_ = true;

  ros::spinOnce();
  ROS_INFO_STREAM("Dropping on AGV...");

  if (gripper_state_){
    auto temp_pose = part_pose;
    temp_pose.position.z += 0.5;
    this->goToTarget({temp_pose, part_pose});
    ros::Duration(1).sleep();
    ros::spinOnce();

    // if (!gripper_state_){
    //         ROS_INFO_STREAM("Fail Check drop part");
    //       this->gripperToggle(true);
    //         ros::spinOnce();
    //       return gripper_state_;
    //     }
    ROS_INFO_STREAM("Actuating the gripper...");
    this->gripperToggle(false);

    ros::spinOnce();
    if (!gripper_state_) {
      std::vector<double> drop_pose_;
	  if(part_pose.position.y>0)
	  drop_pose_ = {0.5, 1.7, -1.1, 1.9, 3.9, 4.7, 0};
	  else
	  drop_pose_ = {-0.5, 4.5, -1.1, 1.9, 3.9, 4.7, 0};
	  sendRobot(drop_pose_);
      // ROS_INFO_STREAM("Going to home...");
      // this->goToTarget({temp_pose, home_cart_pose_});
      // ros::Duration(3.0).sleep();
    }
  }

  

  
  drop_flag_ = false;
  return gripper_state_;
}

void UR10Controller::gripperCallback(
    const osrf_gear::VacuumGripperState::ConstPtr& grip) {
  gripper_state_ = grip->attached;
  // if (drop_flag_ && !gripper_state_) {
  //   robot_move_group_.stop();
  // }
}

// void UR10Controller::gripper_state_check(geometry_msgs::Pose pose) {
//   ros::spinOnce();
//   // ros::Duration(1.0).sleep();

//   if (pick == true) {
//     if (gripper_state == true) {
//       ROS_INFO_STREAM("Part Attached");
//     } else {
//       ROS_INFO_STREAM("Part Not Attached");
//       auto temp_pose_1 = pose;
//       temp_pose_1.position.z += 0.1;
//       auto temp_pose_2 = pose;
//       pose.position.z = pose.position.z - 0.015;
//       this->goToTarget({temp_pose_1, temp_pose_2, pose});
//       this->gripper_state_check(pose);
//     }
//   }

//   else {
//     if (gripper_state == true) {
//       ROS_INFO_STREAM("Dropping: Part Attached");
//       drop = true;
//     } else {
//       ROS_INFO_STREAM("Dropping: Part Not Attached");
//       drop = false;
//     }
//   }
// }

bool UR10Controller::pickPart(geometry_msgs::Pose& part_pose) {
  // gripper_state = false;
  // pick = true;
  // sendRobotHome();
  auto a = part_pose.position.y;
  std::vector<double> robot_pose_ = {a, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
  sendRobot(robot_pose_);

  ROS_INFO_STREAM("fixed_orientation_" << fixed_orientation_.x << fixed_orientation_.y << fixed_orientation_.z << fixed_orientation_.w );
  part_pose.orientation = fixed_orientation_;
  ROS_WARN_STREAM("Picking the part...");

  ROS_INFO_STREAM("Moving to part...");
  part_pose.position.z = part_pose.position.z + offset_;
  auto temp_pose_1 = part_pose;
  temp_pose_1.position.z += 0.35;
  this->goToTarget({part_pose});
  // this->goToTarget({temp_pose_1, part_pose});

  // this->goToTarget(part_pose);
  ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
  this->gripperToggle(true);
  ros::spinOnce();
  while (!gripper_state_) {
    part_pose.position.z -= 0.01;
    this->goToTarget({temp_pose_1, part_pose});
    ROS_INFO_STREAM("Actuating the gripper...");
    this->gripperToggle(true);
    ros::spinOnce();
  }
  // this->gripper_state_check(part_pose);

  // this->gripper_state_check(part_pose);
  // pick = false;
  // auto temp_pose_2 = agv_position_;
  // temp_pose_2.position.z += 0.35;
  ROS_INFO_STREAM("Going to waypoint...");
  this->goToTarget(temp_pose_1);
  // this->goToTarget({temp_pose_2});
  return gripper_state_;
}