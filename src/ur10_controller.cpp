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
  conv_joint_pose_ = {0.0, 1.1, -1.1, 1.9, 3.9, 4.7, 0};
  offset_ = 0.025;

  temp1 = {1.23, 0.6, -0.8, 1.8, 3.9, 4.7, 0};
  temp2 = {1.21, 1.30, -0.50, 0.43, 4.0, -1.57, 0.51};

  gripper_subscriber_ = gripper_nh_.subscribe(
      "/ariac/gripper/state", 10, &UR10Controller::gripperCallback, this);
  joint_subscriber_ = ur10_nh_.subscribe("/ariac/arm/state", 10,
                                         &UR10Controller::jointCallback2, this);
  joint_publisher_ = ur10_nh_.advertise<trajectory_msgs::JointTrajectory>(
      "/ariac/arm/command", 10);
  quality1_subscriber_ =
      ur10_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
                         &UR10Controller::quality1Callback, this);
  quality2_subscriber_ =
      ur10_nh_.subscribe("/ariac/quality_control_sensor_2", 10,
                         &UR10Controller::quality2Callback, this);
  arm_state_subscriber_ = ur10_nh_.subscribe(
      "/ariac/joint_states", 10, &UR10Controller::jointCallback1, this);

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

void UR10Controller::jointCallback1(
    const sensor_msgs::JointState::ConstPtr& joint) {
  joint_state_ = *joint;
}

void UR10Controller::jointCallback2(
    const control_msgs::JointTrajectoryControllerState::ConstPtr& joint_msg) {
  curr_joint_states_ = *joint_msg;
}

void UR10Controller::gripperCallback(
    const osrf_gear::VacuumGripperState::ConstPtr& grip) {
  gripper_state_ = grip->attached;
}

void UR10Controller::quality1Callback(
    const osrf_gear::LogicalCameraImage::ConstPtr& quality1_) {
  quality_1 = *quality1_;
}

void UR10Controller::quality2Callback(
    const osrf_gear::LogicalCameraImage::ConstPtr& quality2_) {
  quality_2 = *quality2_;
}

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

void UR10Controller::execute() {
  ros::AsyncSpinner spinner(4);
  spinner.start();
  if (this->planner()) {
    robot_move_group_.move();
    ros::Duration(0.5).sleep();
  }
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
    q = {i.orientation.x, i.orientation.y, i.orientation.z, i.orientation.w};
    m.setRotation(q);
    m.getRPY(r_h, p_h, y_t);
    q = {fixed_orientation_.x, fixed_orientation_.y, fixed_orientation_.z,
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
    auto value = robot_move_group_.computeCartesianPath(waypoints, 0.001, 0.0,
                                                        traj, true);
    robot_planner_.trajectory_ = traj;
    robot_move_group_.execute(robot_planner_);
    ros::Duration(1.5).sleep();
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

void UR10Controller::sendRobot(std::vector<double> pose) {
  // ros::Duration(2.0).sleep();
  // robot_move_group_.setJointValueTarget(pose);
  // // this->execute();
  // ros::AsyncSpinner spinner(4);
  // spinner.start();
  // if (this->planner()) {
  //   robot_move_group_.move();
  //   ros::Duration(0.5).sleep();
  // }
  // ros::Duration(2.0).sleep();
  trajectory_msgs::JointTrajectory target;
  target.header.stamp = ros::Time::now();
  target.joint_names = curr_joint_states_.joint_names;
  // target.joint_names.pop_back();
  target.points.resize(1);
  target.points[0].positions.resize(target.joint_names.size());
  for (int i = 0; i < target.joint_names.size(); ++i) {
    // ROS_INFO_STREAM("tar: " << tar[i]);
    target.points[0].positions[i] = pose[i];
  }
  target.points[0].time_from_start = ros::Duration(1.0);
  joint_publisher_.publish(target);
  ros::Duration(1.0).sleep();
}

void UR10Controller::gripperToggle(const bool& state) {
  gripper_service_.request.enable = state;
  gripper_client_.call(gripper_service_);
  ros::Duration(0.5).sleep();
  if (gripper_service_.response.success) {
    ROS_INFO_STREAM("Gripper activated!");
  } else {
    ROS_WARN_STREAM("Gripper activation failed!");
  }
}

bool UR10Controller::dropPart(geometry_msgs::Pose part_pose) {
  drop_flag_ = true;
  auto temp_pose = part_pose;
  auto temp_pose2 = part_pose;

  ros::spinOnce();
  ros::Duration(0.1).sleep();

  ROS_INFO_STREAM("Dropping on AGV...");
  bool result = gripper_state_;

  if (gripper_state_) {
    // Turning Right or Left After Picking Up in Joint Angle
    auto angle_pos_ = joint_state_.position;

    if (part_pose.position.y > 0 && angle_pos_[1] > 0)
      angle_pos_ = {angle_pos_[1], angle_pos_[3] + 3.14 + 1.57,
                    -1.1,          1.9,
                    angle_pos_[4], angle_pos_[5],
                    angle_pos_[6]};
    else if (part_pose.position.y > 0 && angle_pos_[1] < 0)
      angle_pos_ = {angle_pos_[1], angle_pos_[3] - 1.57, -1.1,         1.9,
                    angle_pos_[4], angle_pos_[5],        angle_pos_[6]};
    else if (part_pose.position.y < 0 && angle_pos_[1] > 0)
      angle_pos_ = {angle_pos_[1], angle_pos_[3] + 1.57, -1.1,         1.9,
                    angle_pos_[4], angle_pos_[5],        angle_pos_[6]};
    else if (part_pose.position.y < 0 && angle_pos_[1] < 0)
      angle_pos_ = {angle_pos_[1], angle_pos_[3] - 3.14 - 1.57,
                    -1.1,          1.9,
                    angle_pos_[4], angle_pos_[5],
                    angle_pos_[6]};

    sendRobot(angle_pos_);

    if (part_pose.position.y > 0)
      angle_pos_ = {2.2,           1.7,           angle_pos_[2], angle_pos_[3],
                    angle_pos_[4], angle_pos_[5], angle_pos_[6]};
    else
      angle_pos_ = {-2.2,          4.5,           angle_pos_[2], angle_pos_[3],
                    angle_pos_[4], angle_pos_[5], angle_pos_[6]};

    sendRobot(angle_pos_);

    bool flg = false;

    if (!gripper_state_) {
      goto label;
    }

    // For Quality CHeck

    temp_pose.position.z += 0.3;
    this->goToTarget({temp_pose});

    ros::spinOnce();
    ros::Duration(0.1).sleep();

    if (!gripper_state_) {
      flg = true;
      goto label;
    }

    temp_pose2.position.z += 0.01;
    this->goToTarget({temp_pose2, part_pose});

  label:
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    // Writing code for Quality Sensor
    ROS_WARN_STREAM("Output of Quality 1" << quality_1);
    ROS_WARN_STREAM("Output of Quality 2" << quality_2);

    ros::Duration(0.1).sleep();

    if (!quality_1.models.empty() || !quality_2.models.empty() || flg) {
      this->goToTarget({temp_pose});
      std::vector<double> drop_pose_;
      if (part_pose.position.y > 0)
        drop_pose_ = {0.5, 1.7, -1.1, 1.9, 3.9, 4.7, 0};
      else
        drop_pose_ = {-0.5, 4.5, -1.1, 1.9, 3.9, 4.7, 0};
      sendRobot(drop_pose_);
      this->gripperToggle(false);

      ROS_INFO_STREAM("End Of DROP Part with faulty part detected");
      result = true;
    } else {
      ROS_INFO_STREAM("Actuating the gripper in Drop Part");
      this->gripperToggle(false);

      ros::spinOnce();
      if (!gripper_state_) {
        std::vector<double> drop_pose_;
        if (part_pose.position.y > 0)
          drop_pose_ = {0.5, 1.7, -1.1, 1.9, 3.9, 4.7, 0};
        else
          drop_pose_ = {-0.5, 4.5, -1.1, 1.9, 3.9, 4.7, 0};
        sendRobot(drop_pose_);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        result = false;
      }
    }
  }

  return result;
}

bool UR10Controller::pickPart(geometry_msgs::Pose& part_pose) {
  ROS_WARN_STREAM("In Pick Part Function");
  // gripper_state = false;
  // pick = true;
  // sendRobotHome();
  // auto f = flipPart(part_pose);
  auto a = part_pose.position.y;
  std::vector<double> robot_pose_ = {a, 3.1, -1.1, 1.9, 3.9, 4.7, 0};
  sendRobot(robot_pose_);

  ROS_INFO_STREAM("fixed_orientation_"
                  << fixed_orientation_.x << fixed_orientation_.y
                  << fixed_orientation_.z << fixed_orientation_.w);

  ROS_INFO_STREAM("Moving to part...");
  // part_pose.position.z = part_pose.position.z + offset_;
  auto temp_pose_1 = part_pose;
  temp_pose_1.position.z += 0.35;  // changed from 0.35
  // this->goToTarget({part_pose});
  this->goToTarget({part_pose});

  // this->goToTarget(part_pose);
  ROS_INFO_STREAM("Actuating the gripper in Pick Part" << part_pose.position.z);
  this->gripperToggle(true);
  ros::spinOnce();
  ros::Duration(0.5).sleep();

  while (!gripper_state_) {
    part_pose.position.z -= 0.008;  // changed from 0.01
    this->goToTarget({temp_pose_1, part_pose});
    ROS_INFO_STREAM("Actuating the gripper in pick part");
    this->gripperToggle(true);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO_STREAM("Going to waypoint...");
  this->goToTarget({temp_pose_1});
  return gripper_state_;
}

void UR10Controller::publishJoints(const std::vector<double>& tar) {
  trajectory_msgs::JointTrajectory target;
  target.header.stamp = ros::Time::now();
  target.joint_names = curr_joint_states_.joint_names;
  // target.joint_names.pop_back();
  target.points.resize(1);
  target.points[0].positions.resize(target.joint_names.size());
  ROS_WARN_STREAM("here: names done");
  for (int i = 0; i < target.joint_names.size(); ++i) {
    ROS_INFO_STREAM("iter: " << i);
    // ROS_INFO_STREAM("tar: " << tar[i]);
    target.points[0].positions[i] = tar[i];
  }

  ROS_WARN_STREAM("here: points");
  target.points[0].time_from_start = ros::Duration(1.0);
  ROS_WARN_STREAM("here: time");
  joint_publisher_.publish(target);
  ros::Duration(1.0).sleep();
}

bool UR10Controller::flipPart(geometry_msgs::Pose& part_pose) {
  ROS_WARN_STREAM("Start of FLip Part");

  // this->pickPart(part_pose);
  auto temp = part_pose;
  tf::Quaternion q = {temp.orientation.x, temp.orientation.y,
                      temp.orientation.z, temp.orientation.w};
  tf::Matrix3x3 m;
  double roll, pitch, yaw;
  m.setRotation(q);
  m.getRPY(roll, pitch, yaw);

  ros::spinOnce();
  ros::Duration(0.1).sleep();

  auto angle_pos_ = joint_state_.position;
  angle_pos_ = {angle_pos_[1], angle_pos_[3], angle_pos_[2], angle_pos_[0],
                angle_pos_[4], 1.5707,        angle_pos_[6]};

  sendRobot(angle_pos_);

  ROS_INFO_STREAM("reaching temp 1");
  ros::Duration(1.0).sleep();
  this->gripperToggle(false);

  angle_pos_ = {angle_pos_[0], angle_pos_[1], angle_pos_[2], angle_pos_[3],
                angle_pos_[4], -1.5707,       angle_pos_[6]};

  sendRobot(angle_pos_);

  this->gripperToggle(true);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  return gripper_state_;
}

bool UR10Controller::goToConveyor(const geometry_msgs::Pose& part_pose) {
  ros::Time curr_time = ros::Time::now();
  ros::spinOnce();
  auto conv_pose = curr_joint_states_.actual.positions;
  // conv_pose[1] = 0.5;
  // conv_pose[0] = part_pose.position.y;

  this->sendRobot(conv_joint_pose_);
  this->gripperToggle(true);

  auto model = robot_move_group_.getRobotModel();
  auto group = model->getJointModelGroup("manipulator");
  robot_state::RobotStatePtr kinematic_state(
      new robot_state::RobotState(model));

  Eigen::Vector3d ref_position(0.0, 0.0, 0.0);
  Eigen::VectorXd error = Eigen::VectorXd::Zero(7);
  Eigen::MatrixXd jacobian;

  double time_diff;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  target_pose_.position = part_pose.position;
  target_pose_.position.z += 1.1 * offset_;
  target_pose_.position.y += 1.0 * 0.2;

  ros::Time prev_time;
  while (target_pose_.position.y > -2.1 && !gripper_state_) {
    prev_time = curr_time;
    curr_time = ros::Time::now();
    time_diff = curr_time.toSec() - prev_time.toSec();

    // target_pose_.position.y += (conv_camera_.getVelocity() * time_diff);
    ros::spinOnce();
    conv_pose = curr_joint_states_.actual.positions;
    kinematic_state->setJointGroupPositions(group, conv_pose);
    kinematic_state->update();
    kinematic_state->getJacobian(group,
                                 kinematic_state->getLinkModel("ee_link"),
                                 ref_position, jacobian, true);

    auto effector = kinematic_state->getGlobalLinkTransform("ee_link");
    Eigen::Quaterniond effector_quat(effector.rotation());

    error(0) = effector.translation()[0] - target_pose_.position.x;
    error(1) = effector.translation()[1] - target_pose_.position.y;
    error(2) =
        std::min(effector.translation()[2] - target_pose_.position.z, 0.25);
    error(3) = effector_quat.x() - home_cart_pose_.orientation.x;
    error(4) = effector_quat.y() - home_cart_pose_.orientation.y;
    error(5) = effector_quat.z() - home_cart_pose_.orientation.z;
    error(6) = effector_quat.w() - home_cart_pose_.orientation.w;

    svd = jacobian.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd svd_inv(jacobian.cols(), jacobian.rows());
    svd_inv.setZero();

    auto values = svd.singularValues();

    for (auto index = 0; index < values.size(); ++index) {
      if (values(index) > Eigen::MatrixXd::Scalar(1e-3)) {
        svd_inv(index, index) = Eigen::MatrixXd::Scalar{1.0} / values(index);
      }
    }

    auto update = 0.9 * svd.matrixV() * svd_inv * svd.matrixU().transpose() *
                  (error / time_diff);

    for (size_t count = 0; count < 7; ++count) {
      conv_pose[count] -= update[count];
    }
    this->publishJoints(conv_pose);
  }

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  return gripper_state_;
}
