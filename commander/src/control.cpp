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

//The most helpfull sheet youll meet today :)
//http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "Commander");
  //Define class intace
  tf_tracker instance;

  instance.load_param();
  ros::Duration(5).sleep();

  ROS_INFO("Calinbration begins");

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
    joint_group_positions[3] = -M_PI/8;
    joint_group_positions[4] = M_PI/2;
    joint_group_positions[5] = 0;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(search);
    move_group.move();
    joint_group_positions[0] = M_PI-0.01;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(search);
    move_group.move();
  } while(instance.marker_found[0]!=true && ros::ok());
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
      move_group.setPoseTarget(target_pose,"TCP");
      success = (move_group.plan(go_to_marker) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      move_group.move();
    } while(success=false && ros::ok());

    ROS_INFO("Calibrating based on avg pos");

    do {
      move_group.setPoseTarget(target_pose,"TCP");
      success = (move_group.plan(go_to_marker) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      move_group.move();
    } while( instance.avg_marker_found[0]==false && ros::ok());

    ROS_INFO("Done Calibrating");

    //Conering here :D
    tf2::Quaternion q;

    tf2::Transform corner1;
    tf2::Transform corner2;
    tf2::Transform corner3;
    tf2::Transform corner4;
    tf2::Transform base_marker;
    tf2::Transform look;
    tf2::Transform goal_position;



    corner1.setOrigin( tf2::Vector3(0.1,0.1,0.10045));
    corner1.setRotation(tf2::Quaternion(0,0,0,1));

    corner2.setOrigin( tf2::Vector3(-0.1,0.1,0.10045));
    corner2.setRotation(tf2::Quaternion(0,0,0,1));

    corner3.setOrigin( tf2::Vector3(-0.1,-0.1,0.10045));
    corner3.setRotation(tf2::Quaternion(0,0,0,1));

    corner4.setOrigin( tf2::Vector3(0.1,-0.1,0.10045));
    corner4.setRotation(tf2::Quaternion(0,0,0,1));

    look.setOrigin(tf2::Vector3(0,0,0.25));
    look.setRotation(tf2::Quaternion(0,0,0,1));

    base_marker.setOrigin(tf2::Vector3(instance.avg[0].transform.translation.x, instance.avg[0].transform.translation.y,instance.avg[0].transform.translation.z));
    base_marker.setRotation(tf2::Quaternion(instance.avg[0].transform.rotation.x,instance.avg[0].transform.rotation.y,instance.avg[0].transform.rotation.z,instance.avg[0].transform.rotation.w));

    goal_position = base_marker*look;

    tf2::Transform corner[]={corner1, corner2, corner3, corner4};

    success = false;

    target_pose.position.x = goal_position.getOrigin().x();
    target_pose.position.y = goal_position.getOrigin().y();
    target_pose.position.z = goal_position.getOrigin().z();
    target_pose.orientation.x=goal_position.getRotation().x();
    target_pose.orientation.y=goal_position.getRotation().y();
    target_pose.orientation.z=goal_position.getRotation().z();
    target_pose.orientation.w=goal_position.getRotation().w();

    for (int i = 0; i < 4; i++) {

    goal_position=base_marker*corner[i];

    target_pose.position.x = goal_position.getOrigin().x();
    target_pose.position.y = goal_position.getOrigin().y();
    target_pose.position.z = goal_position.getOrigin().z();
    target_pose.orientation.x=goal_position.getRotation().x();
    target_pose.orientation.y=goal_position.getRotation().y();
    target_pose.orientation.z=goal_position.getRotation().z();
    target_pose.orientation.w=goal_position.getRotation().w();

    do {
      move_group.setPoseTarget(target_pose,"TCP");
      success = (move_group.plan(go_to_marker) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      move_group.move();
    } while(success==false && ros::ok());

    ROS_INFO("Corner %d",i);

    ros::Duration(0.5).sleep();
    }




  ros::spin();
}
