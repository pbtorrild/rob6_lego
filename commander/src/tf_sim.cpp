//std headers
#include <ros/ros.h>
#include <iostream>
#include <math.h>
//tf header
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_ros/Transform.h>
#include <tf/transform_broadcaster.h>
//msg header
#include <geometry_msgs/TransformStamped.h>
#include <vision_lego/TransformRPYStamped.h>

class fuck_me{
private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
public:
  fuck_me():
  tf2_(buffer_),  target_frame_("table")
  {

  }
  void broadcast_frame(vision_lego::TransformRPYStamped marker) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header=marker.header;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.child_frame_id = marker.child_frame_id;
    transformStamped.transform.translation=marker.translation;
    tf2::Quaternion q;
    q.setRPY(marker.orientation.Roll,marker.orientation.Pitch,marker.orientation.Yaw);

    transformStamped.transform.rotation.x=q.x();
    transformStamped.transform.rotation.y=q.y();
    transformStamped.transform.rotation.z=q.z();
    transformStamped.transform.rotation.w=q.w();
    br.sendTransform(transformStamped);
  }
};

//The most helpfull sheet youll meet today :)
//http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_simulator");
  //Define class intace
  fuck_me now;
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<vision_lego::TransformRPYStamped>("data/vision_data", 5);

  for (int i = 0; i < 10; i++) {
    ROS_INFO("Simulation begins in: %d sec",10-i);
    ros::Duration(1).sleep();
  }
  ROS_INFO("Simulation begins NOW");

  ros::Rate r(10);
  //the mesage that shall be sent
  vision_lego::TransformRPYStamped marker;

  //The marker msg
  marker.header.frame_id="table";
  marker.child_frame_id="marker_0";
  marker.translation.x=0.4055;
  marker.translation.y=-0.4;
  marker.translation.z=0.00;
  marker.orientation.Roll=0;
  marker.orientation.Pitch=0;
  marker.orientation.Yaw=M_PI/4;

  int seq_marker;
  while (ros::ok()) {
    marker.header.seq=seq_marker;
    marker.header.stamp=ros::Time::now();
    pub.publish(marker);
    now.broadcast_frame(marker);
    seq_marker++;
    ros::spinOnce();
    r.sleep();
  }


}
