#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include "agile_robotics_industrial_automation/ur10_controller.hpp"

void start_competition(ros::NodeHandle &node) {
  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/start_competition c");

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

  // std::vector<std::string> order =
  // {"logical_camera_1_piston_rod_part_2_frame",
  //                                   "logical_camera_1_piston_rod_part_3_frame",
  //                                   "logical_camera_2_gear_part_1_frame",
  //                                   "logical_camera_2_gear_part_2_frame",
  //                                   "logical_camera_2_gear_part_3_frame"};

  ros::NodeHandle node;

  UR10Controller ur10;

  geometry_msgs::Pose target;
  target.position.x = -0.500000;
  target.position.y = -0.735000;
  target.position.z = 0.724951;
  double part_diff = 0.133;
  double bin_diff = 0.765;

  for (auto i = 0; i < 5; i++) {
    ur10.pickPart(target);
    ROS_INFO("Part picked!!!!!");
    ros::Duration(1.0).sleep();
    ur10.dropPart();
    ROS_INFO("Part dropped!!!");

    if (i == 2) {
      target.position.y += (bin_diff - (2 * part_diff));
    } else {
      target.position.y += part_diff;
    }
    target.position.z -= 0.024;
  }

  // tf::StampedTransform transform;
  // tf::TransformListener listener;

  // for (auto i : order) {
  //   listener.waitForTransform("world", i, ros::Time(0), ros::Duration(10));
  //   listener.lookupTransform("/world", i, ros::Time(0), transform);

  //   target.position.x = transform.getOrigin().x();
  //   target.position.y = transform.getOrigin().y();
  //   target.position.z = transform.getOrigin().z();
  //   // target.orientation = transform.getRotation();

  //   ur10.pickPart(target);
  //   ROS_INFO("Part picked!!!!!");
  //   ros::Duration(1.0).sleep();
  //   ur10.dropPart();
  //   ROS_INFO("Part dropped!!!");
  // }

  return 0;
}