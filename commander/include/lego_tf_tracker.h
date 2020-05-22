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
#include <commander/PoseTCP.h>

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
  std::vector<geometry_msgs::TransformStamped> calibrated_marker;

  //Vector containg weather or not the markers are found
  std::vector<bool> marker_found;
  std::vector<bool> avg_marker_found;
  int test_mode;
  //tf tracker
  tf_tracker():
  tf2_(buffer_),  target_frame_("table")
  {

  }
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<commander::PoseTCP>("data/tcp_location", 100);

  void load_param() {
    ros::param::param<int>("/test_mode", test_mode, 0);
    ros::param::param<int>("/num_markers", num_markers, 14);
    ros::param::param<int>("/avg_gate", avg_gate, 300);
    declare_values();
    ROS_INFO("Parameters are now set");
  }

  void declare_values() {

    avg = decltype(avg)(num_markers);
    latest = decltype(latest)(num_markers);
    calibrated_marker = decltype(calibrated_marker)(num_markers);

    //Vector containg the avg_pos of the markers
    marker_found =decltype(marker_found)(num_markers);
    avg_marker_found = decltype(avg_marker_found)(num_markers);
  }

  void send_data(geometry_msgs::Pose tcp,geometry_msgs::Pose goal) {
    commander::PoseTCP msg;
    msg.tcp_location=tcp;
    msg.goal_location=goal;
    pub.publish(msg);
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

  void locate_tcp(geometry_msgs::Pose goal) {
    //just get the latest value
    geometry_msgs::TransformStamped tcp_location_transform;
    try{
    tcp_location_transform = buffer_.lookupTransform("table", "TCP", ros::Time(0));
    } catch (tf2::TransformException &ex) {
    ROS_WARN("Could NOT transform table to TCP: %s", ex.what());
    }

    //Get as pose
    geometry_msgs::Pose tcp_location = transformToPose(tcp_location_transform);

    send_data(tcp_location,goal);
  }

  void data_extractor(geometry_msgs::Pose start,geometry_msgs::Pose goal) {

    send_data(start,goal);
  }

  geometry_msgs::Pose transformToPose(geometry_msgs::TransformStamped transfrom_in) {
    geometry_msgs::Pose Pose;
    Pose.position.x = transfrom_in.transform.translation.x;
    Pose.position.y = transfrom_in.transform.translation.y;
    Pose.position.z = transfrom_in.transform.translation.z;
    Pose.orientation=transfrom_in.transform.rotation;
    return Pose;
  }


};

#endif
