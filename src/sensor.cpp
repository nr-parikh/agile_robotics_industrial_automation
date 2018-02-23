#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <osrf_gear/Proximity.h>
#include <osrf_gear/TrayContents.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "agile_robotics_industrial_automation/ur10_controller.hpp"
#include <std_srvs/Trigger.h>
#include <tf/tf.h>

void start_competition(ros::NodeHandle & node) 
{

	  ros::ServiceClient start_client =
      node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
       if (!start_client.exists()) 
       {
		    ROS_INFO("Waiting for the competition to be ready...");
		    start_client.waitForExistence();
		    ROS_INFO("Competition is now ready.");
        }
    ROS_INFO("Requesting competition start...");
    std_srvs::Trigger srv;  // Combination of the "request" and the "response".
    start_client.call(srv);  // Call the start Service.
       if (!srv.response.success)
        {  
            ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
         }
   		else 
   		{
   			 ROS_INFO("Competition started!");
  		}
}

void comp(const std_msgs::String::ConstPtr & msg)
{
	if (msg->data == "done" )
    {
      ROS_INFO("Competition ended.");
    }
}

void logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    
      ROS_INFO_STREAM_THROTTLE(2,
      "Logical camera: '" << image_msg->models.size() << "' objects.");
    
    
     for(int i=0;i< image_msg->models.size();i++)
     {

        if ( (image_msg-> models[i].type).compare("piston_part") || (image_msg-> models[i].type).compare("gear_part"))
        {
           ROS_INFO_STREAM_THROTTLE(2,"Part Type:"<< image_msg-> models[1].type);
           ROS_INFO_STREAM_THROTTLE(2,"Pose"<< image_msg-> models[1].pose);
        }
        else
        {
           ROS_INFO("Tray Empty");
        }

      }

  } 	

void tray_callback(const osrf_gear::TrayContents::ConstPtr & tray_msg)
  {
    ROS_INFO_STREAM_THROTTLE(2,
      "Tray Contets: '" << tray_msg-> objects.size() << " Faulty Parts");

    for(int i=0;i< tray_msg->objects.size();i++)
         {

            if ( (tray_msg-> objects[i].type).compare("piston_part") || (tray_msg-> objects[i].type).compare("gear_part"))
            {
               ROS_INFO_STREAM_THROTTLE(2,"Part Type:"<< tray_msg-> objects[i].type);
               ROS_INFO_STREAM_THROTTLE(2,"Pose"<< tray_msg-> objects[i].pose);
            }
            else
            {
               ROS_INFO("Tray Empty");
            }

          }




  } 



int main (int argc, char ** argv)
{
	ros::init(argc, argv, "sensor_node");

	ros::NodeHandle node;

	//MyCompetitionClass comp_class (node);

  	ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,comp);

    ros::Subscriber logical_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_3", 10,logical_camera_callback);

    ros::Subscriber tray_subscriber = node.subscribe(
    "/ariac/trays", 10,tray_callback);
   
    // ros::Subscriber break_beam_subscriber = node.subscribe(
    // "/ariac/break_beam_1_change", 10,break_beam_callback);


    ROS_INFO("Hi");
    start_competition(node);
    ros::spin();

    return 0;
}