//std headers
#include <ros/ros.h>
#include <iostream>
#include <math.h>
//tf header
#include <tf2/LinearMath/Quaternion.h>
//msg header
#include <geometry_msgs/TransformStamped.h>

//The most helpfull sheet youll meet today :)
//http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_simulator");
  //Define class intace
  ros::NodeHandle nh;

  ros::Publisher pub_latest = nh.advertise<geometry_msgs::TransformStamped>("data/markers/latest_transform", 5);
  ros::Publisher pub_avg = nh.advertise<geometry_msgs::TransformStamped>("data/markers/running_avg", 5);

  ROS_INFO("Simulation begins in: 20 sec");
  ros::Duration(5).sleep();
  ROS_INFO("Simulation begins in: 15 sec");
  ros::Duration(5).sleep();
  ROS_INFO("Simulation begins in: 10 sec");
  ros::Duration(5).sleep();
  ROS_INFO("Simulation begins in: 5 sec");
  ros::Duration(5).sleep();
  ROS_INFO("Simulation begins NOW");
  ros::Rate r(10);
  //the mesage that shall be sent
  geometry_msgs::TransformStamped latest;
  geometry_msgs::TransformStamped avg;

  //The Rotation
  tf2::Quaternion q;
  q.setRPY(0,0,0);

  //The latest msg
  latest.header.frame_id="world";
  latest.child_frame_id="marker_0";
  latest.transform.translation.x=0.4;
  latest.transform.translation.y=0.4;
  latest.transform.translation.z=0.06;
  latest.transform.rotation.x=q.x();
  latest.transform.rotation.y=q.y();
  latest.transform.rotation.z=q.z();
  latest.transform.rotation.w=q.w();

  //the avg
  avg=latest;
  avg.child_frame_id="avg_marker_0";

  int seq_latest;
  int seq_avg;
  while (ros::ok()) {
    latest.header.seq=seq_latest;
    latest.header.stamp=ros::Time::now();
    pub_latest.publish(latest);
    if (seq_latest%300==0) {
      avg.header.seq=seq_latest;
      avg.header.stamp=ros::Time::now();
      pub_avg.publish(avg);
    }
    seq_latest++;
    ros::spinOnce();
    r.sleep();
  }


}
