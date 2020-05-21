//std headers
#include <ros/ros.h>
#include <iostream>
//moveit headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//lego headers
#include <lego_actionlib.h>
#include <lego_tf_tracker.h>

//The most helpfull sheet youll meet today :)
//http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "Commander");
  //Define class intace
  tf_tracker data;

  data.load_param();

  ros::Subscriber sub_latest = data.nh.subscribe("data/markers/latest_transform", 1, &tf_tracker::latest_transform,&data);
  ros::Subscriber sub_avg = data.nh.subscribe("data/markers/running_avg", 1, &tf_tracker::running_avg,&data);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Setting up movit");
  //setting up movit
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface setEndEffectorLink("TCP"); //NOtE: THIS CAN BE ANY FRAME ON THE ENDEFFECTOR
  move_group.setPoseReferenceFrame("table");
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w=1;
  move_group.startStateMonitor();

  //get acces to action lib
  lego_actionlib actions;

  /*Seach for markers */
  std::vector<double> joint_group_positions =decltype(joint_group_positions)(6);
  moveit::planning_interface::MoveGroupInterface::Plan Plan;
  ROS_INFO("Searching for markers..");
  while(data.marker_found[0]!=true && ros::ok()){
    Plan=actions.marker_search(move_group.getName());
    move_group.execute(Plan);
  }
  ROS_INFO("Found marker with id: 0");

  //HOW TO ACCES TF DATA FOR THE MARKE IR2 WORLD
  //geometry_msgs::Transform Goal = data.latest[marker_id];
  ROS_INFO("Going to marker");
  Plan=actions.go_to_marker(move_group.getName(),data.latest[0]);
  move_group.execute(Plan);

  ROS_INFO("Calibrating based on avg pos");

  do {
    ros::Duration(3).sleep();
  } while( data.avg_marker_found[0]==false && ros::ok());

  ROS_INFO("Done Calibrating");

  while (ros::ok()) {
    switch (data.test_mode) {
      case 0: //Default go to stick mode
              for (int i = 0; i < 4 && ros::ok(); i++) {
                Plan=actions.go_to_stick(move_group.getName(),i,data.avg[0]);
                move_group.execute(Plan);

                ROS_INFO("Robot is @Calibration stick number %d",i);
                data.locate_tcp(actions.latest_pose);

                ROS_INFO("Waiting .5 sec before moving on");
                ros::Duration(0.5).sleep();
            } break;
      case 1: //Default tf_test
              for (int i = 0; i < 4 && ros::ok(); i++) {
                Plan=actions.go_above_marker(move_group.getName(),i,data.avg[0]);
                move_group.execute(Plan);

                ROS_INFO("Robot is + %d cm above marker",i*2);
                data.locate_tcp(table_pose);

                ROS_INFO("Waiting .5 sec before moving on");
                ros::Duration(0.5).sleep();
              } break;
      case 2: //Default tf_test
              for (int i = 0; i < 4 && ros::ok(); i++){
              int data_send_counter;
                Plan=actions.go_above_marker(move_group.getName(),3,data.avg[0]);
                move_group.execute(Plan);
                while (ros::ok()&&data_send_counter <= 1000) {
                  data.locate_tcp(table_pose);
                  ros::Duration(0.04).sleep();
                }
              } break;
    }
  }

  ros::spin();
}
