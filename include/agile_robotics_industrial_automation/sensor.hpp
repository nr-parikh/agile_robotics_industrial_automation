#pragma once

#include <list>
#include <map>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <osrf_gear/LogicalCameraImage.h>

class Sensor {
 public:
  Sensor();
  ~Sensor();
  void camera1Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
  void camera2Callback(
      const osrf_gear::LogicalCameraImage::ConstPtr& image_msg);
  geometry_msgs::Pose getPartPose(const std::string& src_frame,
                                  const std::string& target_frame);
  std::map<std::string, std::list<std::string>> getParts();
  void scanParts(int cam_number);

 private:
  ros::NodeHandle sensor_nh_;
  ros::Subscriber camera_1_subscriber_;
  ros::Subscriber camera_2_subscriber_;

  tf::TransformListener camera_tf_listener_;
  tf::StampedTransform camera_tf_transform_;

  osrf_gear::LogicalCameraImage current_parts_1_;
  osrf_gear::LogicalCameraImage current_parts_2_;

  std::map<std::string, std::list<std::string>> parts_list_;

  bool init_, cam_1_, cam_2_;
  int counter_1_;
  int counter_2_;
};
