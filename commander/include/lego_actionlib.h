#ifndef LEGO_ACTIONLIB_H
#define LEGO_ACTIONLIB_H

//std headers
#include <ros/ros.h>
#include <iostream>
//moveit headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//tf2 headers
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
//msg headers
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

struct lego_actionlib{
private:
  //initialize state machine
  int search_state_machine;
protected:

public:

  moveit::planning_interface::MoveGroupInterface::Plan marker_search(std::string name){
    moveit::planning_interface::MoveGroupInterface move_group(name);
    //initialize the movement plan
    moveit::planning_interface::MoveGroupInterface::Plan search;
    //initialize the vector containing joint positions
    std::vector<double> joint_group_positions =decltype(joint_group_positions)(6);

    bool planning_success;
    do {
      switch (search_state_machine) {
        case 0: joint_group_positions[0] = M_PI-0.01;
                joint_group_positions[1] = -M_PI/2;
                joint_group_positions[2] = -M_PI/2;
                joint_group_positions[3] = -M_PI/8;
                joint_group_positions[4] = M_PI/2;
                joint_group_positions[5] = 0;
                move_group.setJointValueTarget(joint_group_positions);
                planning_success = (move_group.plan(search) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (planning_success==false) {
                  search_state_machine=0;
                  ROS_ERROR("Planning failed miserably :(");
                  break;
                } else{
                  search_state_machine=1;
                  return search;
                }

        case 1: joint_group_positions[0] = -M_PI+0.01;
                joint_group_positions[1] = -M_PI/2;
                joint_group_positions[2] = -M_PI/2;
                joint_group_positions[3] = -M_PI/8;
                joint_group_positions[4] = M_PI/2;
                joint_group_positions[5] = 0;
                move_group.setJointValueTarget(joint_group_positions);
                planning_success = (move_group.plan(search) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if (planning_success==false) {
                  search_state_machine=1;
                  ROS_ERROR("Planning failed miserably :(");
                  break;
                } else{
                  search_state_machine=0;
                  return search;
                }
      }
    } while(planning_success!=true && ros::ok());
  }

  moveit::planning_interface::MoveGroupInterface::Plan go_to_marker(std::string name,geometry_msgs::TransformStamped marker){
    moveit::planning_interface::MoveGroupInterface move_group(name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //get marker pose
    geometry_msgs::Pose marker_pose=transformToPose(marker);

    bool planning_success;
    do {
      move_group.setJointValueTarget(marker_pose,"TCP");
      planning_success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (planning_success==false) {
        ROS_ERROR("Planning failed miserably :(");
        break;
      } else{
        return plan;
      }
    } while(planning_success!=true && ros::ok());

  }
  moveit::planning_interface::MoveGroupInterface::Plan go_to_stick(std::string name,int stick_num ,geometry_msgs::TransformStamped marker){
    moveit::planning_interface::MoveGroupInterface move_group(name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //get marker pose
    geometry_msgs::Pose marker_pose;
    switch (stick_num) {
      case 0: marker_pose=stickLocation(0.10,0.10,0.10045,marker); break;
      case 1: marker_pose=stickLocation(-0.10,0.10,0.10045,marker); break;
      case 2: marker_pose=stickLocation(0.10,-0.10,0.10045,marker); break;
      case 3: marker_pose=stickLocation(-0.10,-0.10,0.10045,marker); break;
    }

    bool planning_success;
    do {
      move_group.setJointValueTarget(marker_pose,"TCP");
      planning_success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      if (planning_success==false) {
        ROS_ERROR("Planning failed miserably :(");
        break;
      } else{
        return plan;
      }
    } while(planning_success!=true && ros::ok());

  }

  geometry_msgs::Pose transformToPose(geometry_msgs::TransformStamped transfrom_in) {
    geometry_msgs::Pose Pose;
    Pose.position.x = transfrom_in.transform.translation.x;
    Pose.position.y = transfrom_in.transform.translation.y;
    Pose.position.z = transfrom_in.transform.translation.z;
    Pose.orientation=transfrom_in.transform.rotation;
    return Pose;
  }
  geometry_msgs::Pose stickLocation(double x,double y,double z,geometry_msgs::TransformStamped transfrom_in){
    //Get RPY from teh transform
    tf2::Quaternion q(transfrom_in.transform.rotation.x,transfrom_in.transform.rotation.y,transfrom_in.transform.rotation.z,transfrom_in.transform.rotation.w);
    double R, P, Y;
    tf2::Matrix3x3 matrix(q);
    matrix.getRPY(R, P, Y);
    // rotation angle about X-axis (pitch)
    double sin_P = sin(P);
    double cos_P = cos(P);
    // rotation angle about Y-axis (yaw)
    double sin_Y = sin(Y);
    double cos_Y = cos(Y);
    // rotation angle about Z-axis (roll)
    double sin_R = sin(R);
    double cos_R = cos(R);
    //calculate the new translation
    x=cos_R*cos_Y*x-sin_R*cos_P*y+cos_R*sin_Y*sin_P*y+sin_R*sin_P*z+cos_R*sin_Y*cos_P*z;
    y=sin_R*cos_Y*x+cos_R*cos_P*y+cos_R*sin_Y*sin_P*y-cos_R*sin_P*z+sin_R+sin_Y+cos_P*z;
    z=-sin_Y*x+cos_Y*sin_P*y+cos_Y*cos_P*z;
    //initialize return pose
    geometry_msgs::Pose Pose;
    Pose.position.x = x;
    Pose.position.y = y;
    Pose.position.z = z;
    Pose.orientation=transfrom_in.transform.rotation;
    return Pose;
  }

};



#endif
