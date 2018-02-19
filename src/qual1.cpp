#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include "agile_robotics_industrial_automation/ur10_controller.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ariac_example_node");

  ros::NodeHandle node;

  UR10Controller ur10;

  geometry_msgs::Pose target;

  // auto quaternion = tf::createQuaternionFromRPY(-0.12,0.059,1.64);

  // target.orientation.x = quaternion[0];
  // target.orientation.y = quaternion[1];
  // target.orientation.z = quaternion[2];
  // target.orientation.w = quaternion[3];

  // we set positions for our pose
  target.position.x = -0.5;
  target.position.y = -0.735;
  target.position.z = 0.724;
  // ur10.setTarget(target);
  // ROS_INFO("Moving");
  // ur10.execute();

  ur10.pickPart(target);
  ROS_INFO("Part picked!!!!!");
  ros::Duration(1.0).sleep();
  ur10.dropPart();
  ROS_INFO("Part dropped!!!");

  // UR10_JointControl ur10(node);

  // ur10.gripperAction(gripper::OPEN);

  // ur10.jointPosePublisher({1.85, 0.35, -0.38, 2.76, 3.67, -1.51, 0.00});

  // ur10.gripperAction(gripper::CLOSE);

  // ros::spin();

  return 0;
}