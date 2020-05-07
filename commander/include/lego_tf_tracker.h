#ifndef LEGO_TF_TRACKER_H
#define LEGO_TF_TRACKER_H

#include <math.h>
#include <ros/ros.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <vision_lego/TransformRPYStamped.h>

struct tf_tracker{
private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  //the number of times a maker have to be seen inorder for the avg to be taken
  int avg_gate;
  int num_markers;


protected:

public:
  //Two vectirs containng the transposes of the running average and the latest markers
  std::vector<geometry_msgs::TransformStamped> avg; //Create vector of ids containg a vector of size 4 each containing a cv point 3f
  std::vector<geometry_msgs::TransformStamped> latest;

  //Vector containg weather or not the markers are found
  std::vector<bool> marker_found;
  std::vector<bool> avg_marker_found;

  //tf tracker
  tf_tracker():
  tf2_(buffer_),  target_frame_("world")
  {

  }
  ros::NodeHandle nh;

  void load_param() {
    ros::param::param<int>("/num_markers", num_markers, 14);
    ros::param::param<int>("/avg_gate", avg_gate, 300);
    declare_values();
    ROS_INFO("Parameters are now set");
  }

  void declare_values() {

    avg = decltype(avg)(num_markers);
    latest = decltype(avg)(num_markers);

    //Vector containg the avg_pos of the markers
    marker_found =decltype(marker_found)(num_markers);
    avg_marker_found = decltype(avg_marker_found)(num_markers);
  }


  void running_avg(geometry_msgs::TransformStamped msg){

   //Get frame id as int
   std::string frame_id =msg.child_frame_id;
   frame_id.erase(0,11);
   int id_num = std::stoi(frame_id);
   //read avg translation
   avg[id_num]=msg;
   avg_marker_found[id_num] = true;

 }
  void latest_transform(geometry_msgs::TransformStamped msg){

   //Get frame id as int
   std::string frame_id =msg.child_frame_id;
   frame_id.erase(0,7);
   int id_num = std::stoi(frame_id);
   //read latest translation
   latest[id_num]=msg;
   marker_found[id_num] = true;

  }


};

#endif
