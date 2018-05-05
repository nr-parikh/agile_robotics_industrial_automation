
#include <string>
#include <ros/ros.h>
#include <osrf_gear/TrayContents.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// public bool flag=0;

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
      int k=0; int part_num=5;
    
      ROS_INFO_STREAM("Logical camera: '" << image_msg->models.size() << "' objects.");

      tf::StampedTransform part_tf;
      tf::TransformListener part_tf_listener;
    
     for(int i=2;i<image_msg->models.size();i++)
     {


         std::string part_frame = "/logical_camera_3_"+image_msg-> models[i].type+"_"+ std::to_string(part_num)+ "_frame";
         std::string  world_frame = "/world";
        part_tf_listener.waitForTransform(world_frame, part_frame, ros::Time(0), ros::Duration(10));
        part_tf_listener.lookupTransform(world_frame, part_frame, ros::Time(0), part_tf);
         auto  x = part_tf.getOrigin().x();
         auto y = part_tf.getOrigin().y();
        auto  z = part_tf.getOrigin().z();

        if(part_num==5)
        {
           ROS_INFO("Faulty Part Detected");
           ROS_INFO_STREAM("Part Type:"<< image_msg-> models[i].type);
           ROS_INFO_STREAM("Posex"<< x);
           ROS_INFO_STREAM("Posey"<< y);
           ROS_INFO_STREAM("Posez"<< z);
       }
         else
         {
		   ROS_INFO("Normal Part Detected");
           ROS_INFO_STREAM("Part Type:"<< image_msg-> models[i].type);
           ROS_INFO_STREAM("Posex"<< x);
           ROS_INFO_STREAM("Posey"<< y);
           ROS_INFO_STREAM("Posez"<< z);
       }

 
//how is it taking pose and orientation of logical camera.
          ROS_INFO_STREAM("End of Loop"<< i);
           k=1;

      }
      if (k==0)
      {
        ROS_INFO("Empty Tray");
      }

  } 	

// void tray_callback(const osrf_gear::TrayContents::ConstPtr & tray_msg)
//   {
//     ROS_INFO_STREAM_THROTTLE(2,
//       "Tray Contets: '" << tray_msg-> objects.size() << " Faulty Parts");

//     for(int i=0;i< tray_msg->objects.size();i++)
//          {

//             if ( (tray_msg-> objects[i].type).compare("piston_part") || (tray_msg-> objects[i].type).compare("gear_part"))
//             {
//                ROS_INFO_STREAM_THROTTLE(2,"Part Type:"<< tray_msg-> objects[i].type);
//                ROS_INFO_STREAM_THROTTLE(2,"Pose"<< tray_msg-> objects[i].pose);
//             }
//             else
//             {
//                ROS_INFO("Tray Empty");
//             }

//           }

  // } 


void quality_callback(const osrf_gear::LogicalCameraImage::ConstPtr & quality_msg)
{
      if (quality_msg-> models.size()==0)
      {
      	ROS_INFO("No Faulty Parts");
      }
     else
     {
      tf::StampedTransform part_tf;
      tf::TransformListener part_tf_listener;
      std::string part_frame = "/logical_camera_3_"+quality_msg-> models[1].type+"_5" + "_frame";
      std::string  world_frame = "/world";
      part_tf_listener.waitForTransform(world_frame, part_frame, ros::Time(0), ros::Duration(10));       part_tf_listener.lookupTransform(world_frame, part_frame, ros::Time(0), part_tf);

        
      auto  x = part_tf.getOrigin().x();
      auto y = part_tf.getOrigin().y();
      auto  z = part_tf.getOrigin().z();
      ROS_INFO_STREAM("Part Type:"<< quality_msg-> models[1].type);
      ROS_INFO_STREAM("Posex"<< x);
      ROS_INFO_STREAM("Posey"<< y);
      ROS_INFO_STREAM("Posez"<< z);

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

   ros::Subscriber quality_sensor_callback = node.subscribe(
    "/ariac/quality_control_sensor_2", 10,quality_callback);


    // ros::Subscriber tray_subscriber = node.subscribe(
    // "/ariac/trays", 10,tray_callback);
   
    // ros::Subscriber break_beam_subscriber = node.subscribe(
    // "/ariac/break_beam_1_change", 10,break_beam_callback);


    ROS_INFO("Hi");
    start_competition(node);
    ros::spin();

    return 0;
}