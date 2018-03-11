
#include "sensor.hpp"

sensor::sensor(ros::NodeHandle &sensor_nh)
{
  ROS_INFO_STREAM("Reading data from logical sensor");
  topic3 = "/ariac/logical_camera_3";
  logical_camera_3_subscriber = sensor_nh.subscribe(topic3, 10,&sensor::logical_camera_3, this);

}

geometry_msgs::Pose sensor::get_part_pose(string logical_camera,string part_name,int part_num)
{
	logical_camera_3_subscriber;
	ros::spinOnce();
	ROS_INFO_STREAM("Getting Pose of required part from logical_camera");
	camera_frame = "/"+logical_camera + "_" + part_name + "_" + std::to_string(part_num)+"_frame";
	world_frame = "/world";
	part_tf_listener.waitForTransform(world_frame, camera_frame, ros::Time(0), ros::Duration(10));
	part_tf_listener.lookupTransform(world_frame, camera_frame, ros::Time(0), part_tf);

	part_pose.position.x = part_tf.getOrigin().x();
	part_pose.position.y = part_tf.getOrigin().y();
	part_pose.position.z = part_tf.getOrigin().z();
    
    return part_pose;
	

}

void sensor::logical_camera_3(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
{
	ROS_INFO_STREAM_THROTTLE(10,
	  "Logical camera 3: '" << image_msg->models.size() << "' objects.");
	ros::Duration(1).sleep();

	// auto parts = image_msg->models;
	// for (int i=0; i < parts.size();i++)
	// {
	// 	ROS_INFO_STREAM("Part_pose_camera_frame:\n" << parts[i].pose.position);
	// 	if(parts[i].type == order_part){
	// 		this->get_part_pose(parts[i].pose);
	// 		break;
	// 	}

	// 	ros::Duration(0.5).sleep();
	// }
	
	// ros::Duration(5).sleep();
}

// void sensor::logical_camera_1(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
// {
// 	ROS_INFO_STREAM_THROTTLE(10,
// 	  "Logical camera 2: '" << image_msg->models.size() << "' objects.");
// 	ros::Duration(1).sleep();

// }
int main(int argc, char **argv) 
{
  ros::init(argc, argv, "quality_node");
  ros::NodeHandle sensor_nh;
  sensor camera_sensor(sensor_nh);
  geometry_msgs::Pose pose = camera_sensor.get_part_pose("logical_camera_1","piston_rod_part",3);
  ROS_INFO_STREAM("Part_pose_world_frame:\n" << pose.position);
  ros::spin();  

  return 0;

}