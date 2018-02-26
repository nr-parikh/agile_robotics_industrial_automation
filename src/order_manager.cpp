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
      // ROS_WARN_STREAM("object.type:>>>>>>>>>>" << object.type << "typeid>>>>>>"
      //                                          << typeid(object.type).name());
    }
  }
}

std::string OrderManager::getPartType(std::string object) {
  // ROS_WARN_STREAM("<<<<<<<>>>>>>>>>>"
  //                 << object << ">>>>>>>>>>type:" << typeid(object).name());
  std::string part = scanned_objects_[object].front();
  scanned_objects_[object].pop_front();
  return part;
}

void OrderManager::executeOrder() {
  ROS_INFO_STREAM("Executing order...");
  geometry_msgs::Pose part_pose;
  bool success;

  ros::spinOnce();
  ros::Duration(3.0).sleep();

  for (auto& kit : order_) {
    ROS_INFO_STREAM(">>>>>>>>" << kit.first);
    std::list<std::string> parts = kit.second;
    std::string object;
    while (!parts.empty()) {
      object = this->getPartType(kit.first);
      part_pose = camera_.getPartPose("/world", object);
      part_pose.position.z = part_pose.position.z + 0.075;
      robot_.pickPart(part_pose);
      parts.pop_front();
      ROS_INFO_STREAM("Part picked: " << object);
      success = robot_.dropPart();
      if (!success) {
        parts.push_front(this->getPartType(kit.first));
      }
    }

    ROS_INFO_STREAM("Part dropped: " << object);
  }
}
