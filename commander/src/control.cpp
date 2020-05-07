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

#include <lego_actionlib.h>

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
  tf_tracker tf_data;

  tf_data.load_param();
  ros::Duration(5).sleep();

  ROS_INFO("Calinbration begins");

  ros::Subscriber sub_latest = tf_data.nh.subscribe("data/markers/latest_transform", 1, &tf_tracker::latest_transform,&tf_data);
  ros::Subscriber sub_avg = tf_data.nh.subscribe("data/markers/running_avg", 1, &tf_tracker::running_avg,&tf_data);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  //setting up movit
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface setEndEffectorLink("TCP"); //NOtE: THIS CAN BE ANY FRAME ON THE ENDEFFECTOR
  move_group.setPoseReferenceFrame("world");
  move_group.startStateMonitor();

  //get acces to action lib
  lego_actionlib actions;

  /*Seach for markers */
  std::vector<double> joint_group_positions =decltype(joint_group_positions)(6);
  moveit::planning_interface::MoveGroupInterface::Plan Plan;
  ROS_INFO("Searching for markers..");
  do {
    Plan=actions.marker_search(move_group.getName());
    move_group.execute(Plan);
  } while(tf_data.marker_found[0]!=true && ros::ok());
  ROS_INFO("Found marker with id: 0");

  //HOW TO ACCES TF DATA FOR THE MARKE IR2 WORLD
  //geometry_msgs::Transform Goal = tf_data.latest[marker_id];
  ROS_INFO("Going to marker");
  Plan=actions.go_to_marker(move_group.getName(),tf_data.latest[0]);
  move_group.execute(Plan);

  ROS_INFO("Calibrating based on avg pos");

  do {
    ros::Duration(3).sleep();
  } while( tf_data.avg_marker_found[0]==false && ros::ok());

  ROS_INFO("Done Calibrating");

  Plan=actions.go_to_stick(move_group.getName(),0,tf_data.avg[0]);
  move_group.execute(Plan);

  ROS_INFO("Robot is @Calibration stick number 0");
  ROS_INFO("Waiting 20 sec before moving on");
  ros::Duration(20).sleep();

  Plan=actions.go_to_stick(move_group.getName(),1,tf_data.avg[0]);
  move_group.execute(Plan);

  ROS_INFO("Robot is @Calibration stick number 1");
  ROS_INFO("Waiting 20 sec before moving on");
  ros::Duration(20).sleep();

  Plan=actions.go_to_stick(move_group.getName(),2,tf_data.avg[0]);
  move_group.execute(Plan);

  ROS_INFO("Robot is @Calibration stick number 2");
  ROS_INFO("Waiting 20 sec before moving on");
  ros::Duration(20).sleep();

  Plan=actions.go_to_stick(move_group.getName(),3,tf_data.avg[0]);
  move_group.execute(Plan);

  ROS_INFO("Robot is @Calibration stick number 3");
  ROS_INFO("Waiting 20 sec before moving on");
  ros::Duration(20).sleep();

  ros::spin();
}
