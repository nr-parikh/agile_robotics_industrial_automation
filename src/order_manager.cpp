#include "agile_robotics_industrial_automation/order_manager.hpp"

OrderManager::OrderManager() {
  order_subscriber_ = manager_nh_.subscribe("/ariac/orders", 10,
                                            &OrderManager::orderCallback, this);
  scanned_objects_ = camera_.getParts();
}

OrderManager::~OrderManager() {}

void OrderManager::orderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
  for (auto& kit : order_msg->kits) {
    for (auto& object : kit.objects) {
      order_[object.type].push_back(scanned_objects_[object.type].front());
      scanned_objects_[object.type].pop_front();
    }
  }
}

void OrderManager::executeOrder() {
  ROS_INFO_STREAM("Executing order...");
  geometry_msgs::Pose part_pose;

  ros::spinOnce();
  ros::Duration(3.0).sleep();

  for (auto& kit : order_) {
    ROS_INFO_STREAM(kit.first);
    for (auto& object : kit.second) {
      part_pose = camera_.getPartPose("/world", object);
      part_pose.position.z = part_pose.position.z + 0.1;
      robot_.pickPart(part_pose);
      ROS_INFO_STREAM("Part picked: " << object);
      bool drop = robot_.dropPart();
      ROS_INFO_STREAM("Part dropped: " << object);
      if(drop == false)
      {

        // part_pose = camera_.getPartPose("/world", object);
        // part_pose.position.z = part_pose.position.z + 0.1;
        // robot_.pickPart(part_pose);
        // ROS_INFO_STREAM("Part picked: " << object);
        // drop = robot_.dropPart();
        ROS_INFO_STREAM("Part dropped in the way: " << object);
      }
    }
  }
}