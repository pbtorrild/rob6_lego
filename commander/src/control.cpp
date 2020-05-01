#include <math.h>
#include <ros/ros.h>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_ros/Transform.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <vision_lego/TransformRPYStamped.h>


#include <message_filters/subscriber.h>

class tf_tracker{
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
  ros::Publisher pub_latest = nh.advertise<geometry_msgs::TransformStamped>("data/markers/latest_transform", 5);
  ros::Publisher pub_avg = nh.advertise<geometry_msgs::TransformStamped>("data/markers/running_avg_transform", 5);

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
   frame_id.erase(0,10);
   int id_num = std::stoi(frame_id);
   //read avg translation
   avg[id_num]=msg;
   avg_marker_found[id_num] = true;

 }
  void latest_transform(geometry_msgs::TransformStamped msg){

   //Get frame id as int
   std::string frame_id =msg.child_frame_id;
   frame_id.erase(0,6);
   int id_num = std::stoi(frame_id);
   //read latest translation
   latest[id_num]=msg;
   marker_found[id_num] = true;

  }


};

//The most helpfull sheet youll meet today :)
//http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "Commander");
  //Define class intace
  tf_tracker instance;

  ros::Subscriber sub_latest = instance.nh.subscribe("data/markers/latest_transform", 1, &tf_tracker::latest_transform,&instance);
  ros::Subscriber sub_avg = instance.nh.subscribe("data/markers/running_avg", 1, &tf_tracker::running_avg,&instance);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface setEndEffectorLink("TCP"); //NOtE: THIS CAN BE ANY FRAME ON THE ENDEFFECTOR
  move_group.setPoseReferenceFrame("world");

  move_group.startStateMonitor();
  /*Seach for markers */
  std::vector<double> joint_group_positions =decltype(joint_group_positions)(6);
  moveit::planning_interface::MoveGroupInterface::Plan search;
  ROS_INFO("Searching for markers..");
  do {
    joint_group_positions[0] = 0.01;
    joint_group_positions[1] = -M_PI/2;
    joint_group_positions[2] = -M_PI/2;
    joint_group_positions[3] = -M_PI/4-M_PI/8;
    joint_group_positions[4] = M_PI/2;
    joint_group_positions[5] = 0;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(search);
    move_group.move();
    joint_group_positions[0] = 2*M_PI-0.01;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(search);
    move_group.move();
  } while(instance.marker_found[0]==false && ros::ok());
    ROS_INFO("Found marker with id: 0");

  //HOW TO ACCES TF DATA FOR THE MARKE IR2 WORLD
  //geometry_msgs::Transform Goal = instance.avg_pos[marker_id];
    moveit::planning_interface::MoveGroupInterface::Plan go_to_marker;
    move_group.setGoalOrientationTolerance(0.00001);
    move_group.setGoalPositionTolerance(0.00001 );
    geometry_msgs::Pose target_pose;

    //Move close to marker
    bool success = false;
    target_pose.position.x = instance.latest[0].transform.translation.x;
    target_pose.position.y = instance.latest[0].transform.translation.y;
    target_pose.position.z = instance.latest[0].transform.translation.z+0.25;
    target_pose.orientation.x=instance.latest[0].transform.rotation.x;
    target_pose.orientation.y=instance.latest[0].transform.rotation.y;
    target_pose.orientation.z=instance.latest[0].transform.rotation.z;
    target_pose.orientation.w=instance.latest[0].transform.rotation.w;
    do {
      move_group.setPoseTarget(target_pose,"camera_color_frame");
      success = (move_group.plan(go_to_marker) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      move_group.move();
    } while(success=false && instance.avg_marker_found[0]==false && ros::ok());

    ROS_INFO("Calibrating based on avg pos");

    do {
      move_group.setPoseTarget(target_pose,"TCP");
      move_group.plan(go_to_marker);
      move_group.move();
      target_pose.position.z = instance.avg[0].transform.translation.z+0.10045;
      target_pose.position.y = instance.avg[0].transform.translation.y+0.093;
      target_pose.position.x = instance.avg[0].transform.translation.x+0.1;
      ROS_INFO("Corner 1");
    } while(ros::ok());

      ros::Duration(0.50).sleep();



    do {
      move_group.setPoseTarget(target_pose,"TCP");
      move_group.plan(go_to_marker);
      move_group.move();
      target_pose.position.z = instance.avg[0].transform.translation.z+0.10045;
      target_pose.position.y = instance.avg[0].transform.translation.y-0.1;
      target_pose.position.x = instance.avg[0].transform.translation.x+0.1;
      ROS_INFO("Corner 2");
    }while(ros::ok());

    ros::Duration(0.50).sleep();

    do {
      move_group.setPoseTarget(target_pose,"TCP");
      move_group.plan(go_to_marker);
      move_group.move();
      target_pose.position.z = instance.avg[0].transform.translation.z+0.10045;
      target_pose.position.y = instance.avg[0].transform.translation.y-0.1;
      target_pose.position.x = instance.avg[0].transform.translation.x-0.1;
      ROS_INFO("Corner 3");
    }while(ros::ok());


  ros::Duration(0.50).sleep();

    do {
      move_group.setPoseTarget(target_pose,"TCP");
      move_group.plan(go_to_marker);
      move_group.move();
      target_pose.position.z = instance.avg[0].transform.translation.z+0.10045;
      target_pose.position.y = instance.avg[0].transform.translation.y+0.1;
      target_pose.position.x = instance.avg[0].transform.translation.x-0.1;
      ROS_INFO("Test2 done");
    }while(ros::ok());





  ros::spin();
}
