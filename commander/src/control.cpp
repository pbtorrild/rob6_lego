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

  ROS_INFO("Doing intial calibration");
  do {
    ros::Duration(0.3).sleep();
  } while( data.avg_marker_found[0]==false && ros::ok());

  ROS_INFO("The precise calibration is begon");
  //Go to the position where the calibration is precise
  //here we want to be sure all the data is from this positon therefore we wait
  //the 2 next averages to come in before we are satisfied
  Plan=actions.precision_calibration(move_group.getName(),data.avg[0]);
  move_group.execute(Plan);
  int avg_seq_num = data.avg[0].header.seq;
  int avg_in_counter;
  do {
    ros::Duration(0.3).sleep();
    avg_in_counter=data.avg[0].header.seq;
  } while(ros::ok() && avg_in_counter<avg_seq_num+2);
  data.calibrated_marker[0]=data.avg[0];
  double R,P,Y;
  tf2::Quaternion q(data.calibrated_marker[0].transform.rotation.x,data.calibrated_marker[0].transform.rotation.y,data.calibrated_marker[0].transform.rotation.z,data.calibrated_marker[0].transform.rotation.w);
  tf2::Matrix3x3 matrix(q);
  matrix.getRPY(R,P,Y);

  ROS_INFO("----------- Calibration DONE -----------");
  ROS_INFO(" ");
  ROS_INFO("POSITION:(%F,%F,%F)",data.calibrated_marker[0].transform.translation.x,data.calibrated_marker[0].transform.translation.y,data.calibrated_marker[0].transform.translation.z);
  ROS_INFO("ROTATION:(%F,%F,%F)",R,P,Y);
  ROS_INFO(" ");
  ROS_INFO("----------------------------------------");



  while (ros::ok()) {
    switch (data.test_mode) {
      case 0: //Default go to stick mode
              for (int i = 0; i < 4 && ros::ok(); i++) {
                Plan=actions.go_to_stick(move_group.getName(),i,data.calibrated_marker[0]);
                move_group.execute(Plan);

                ROS_INFO("Robot is @Calibration stick number %d",i);
                data.locate_tcp(actions.latest_pose);

                ROS_INFO("Waiting .5 sec before moving on");
                ros::Duration(0.5).sleep();
            } break;
      case 1: //Default tf_test
              for (int i = 0; i < 4 && ros::ok(); i++) {
                Plan=actions.go_above_marker(move_group.getName(),i,data.calibrated_marker[0]);
                move_group.execute(Plan);

                ROS_INFO("Robot is + %d cm above marker",i*2);
                data.locate_tcp(table_pose);

                ROS_INFO("Waiting .5 sec before moving on");
                ros::Duration(0.5).sleep();
              } break;
      case 2: //Default stationary for cam test
              Plan=actions.go_above_marker(move_group.getName(),2,data.calibrated_marker[0]);
              move_group.execute(Plan);
              while (ros::ok()) {
                data.data_extractor(data.transformToPose(data.latest[0]),table_pose);
                ros::Duration(0.035).sleep();
              }break;
    }
  }

  ros::spin();
}
