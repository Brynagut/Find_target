#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <geometry_msgs/Point.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "find_target/target_position.h"



tf2_ros::Buffer tf_buffer;



using namespace::std;



/* It wasn't obvious at first, but as you see in "callback" I didn't take into account that I couldn't use the point from 
the camera to the object in order for the arm manipulator to make sense out of it. Therefore I used transform_between_frames  */

// find base link and camera position
const std::string from_frame = "camera_depth_optical_frame";
const std::string to_frame = "base_link";

geometry_msgs::Point target_position_base_frame;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;




geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame) {
    
  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.pose.position = p;
  input_pose_stamped.header.frame_id = from_frame;
  input_pose_stamped.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped output_pose_stamped = tf_buffer.transform(input_pose_stamped, to_frame, ros::Duration(1));
  return output_pose_stamped.pose.position;
}  



void callback(const PointCloud::ConstPtr& msg){
  double minDistance=0.0;
  double min_angle_radx=0.0;
  double min_angle_rady=0.0;
  double xX=0.0,yY=0.0,zZ=0.0;
  int count=0;
  // Angles are calculated in radians and can convert to degree by multpying it with 180/pi 
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){//to iterate trough all the points in the filtered point cloud published by publisher
    if(atan2(pt.z, pt.y)*(180/3.14159265358979323846)>80.00){// atan2(z,y)= arctan(z/y) if z>0;
      // truncating points with less that 80 degree vertical angle
      // because the point formed could be ground. 
        if(count==0){
        // initializing the first point read as minimum distance point
        minDistance=hypot(pt.z, pt.x);
        min_angle_radx=atan2(pt.z,pt.x);
        min_angle_rady=atan2(pt.z, pt.y);
        xX=pt.x;
        yY=pt.y;
        zZ=pt.z;
        count++;
        }
       else if(hypot(pt.z, pt.x)<minDistance){
            // keep updating the minimum Distant point
            minDistance=hypot(pt.z, pt.x);
            min_angle_radx=atan2(pt.z,pt.x);
            min_angle_rady=atan2(pt.z, pt.y);
            xX=pt.x;
            yY=pt.y;
            zZ=pt.z;
        }
        else{
          continue;
        }
      }
  }


// uncomment if you want to display data from camera frame
/*
 ROS_INFO_STREAM("Distance="<<minDistance<<"\n");
 ROS_INFO_STREAM("Angle in Degree X axis="<<min_angle_radx*(180/3.14159265358979323846)<<"\n");
 ROS_INFO_STREAM("Angle in Degree Y axis="<<min_angle_rady*(180/3.14159265358979323846)<<"\n");
 ROS_INFO_STREAM("pointXcoordinate="<<xX<<"\n");
 ROS_INFO_STREAM("pointYcoordinate="<<yY<<"\n");
 ROS_INFO_STREAM("pointZcoordinate="<<zZ<<"\n");
 sleep(1); */



// To clean it up I reference the point from which the camear sees as p
 geometry_msgs::Point p;
  p.x = xX;
  p.y = yY;
  p.z = zZ;

    
  target_position_base_frame = transform_between_frames(p, from_frame, to_frame);

 
  ROS_INFO_STREAM("3d bush position base frame: x " << target_position_base_frame.x << " y " << target_position_base_frame.y << " z " << target_position_base_frame.z);
  sleep(1);
} 



// We want to create a service call response to instruct the robot hand
bool get_target_position(find_target::target_position::Request  &req,
    find_target::target_position::Response &res) {
      res.target_position = target_position_base_frame;
      return true;
    } 



int main(int argc, char** argv)
{
  ros::init(argc, argv,"sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<PointCloud>("bush_detected", 1, callback);

  tf2_ros::TransformListener listener(tf_buffer);

  ros::ServiceServer service = n.advertiseService("target_position",  get_target_position);

  ros::spin();
}
