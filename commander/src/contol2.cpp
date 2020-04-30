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

#include <message_filters/subscriber.h>

class tf_tracker{
private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  //the number of times a maker have to bee seen inorder for the avg to be taken

  int num_markers=14;
  //Make a vector to take avg so that:
  //avg[id][0].x = tx
  //avg[id][1].x = rx
  std::vector<cv::Point3f> Point = decltype(Point)(2);
  std::vector<std::vector<cv::Point3f>> avg = decltype(avg)(num_markers,Point); //Create vector of ids containg a vector of size 4 each containing a cv point 3f
  std::vector<int> counter =decltype(counter)(num_markers);

protected:

public:
  std::vector<bool> marker_found =decltype(marker_found)(num_markers);
  bool num_markers_found;
  tf_tracker():
  tf2_(buffer_),  target_frame_("camera_link")
  {

  }
  ros::NodeHandle nh;
  int seen = 1;
  std::vector<geometry_msgs::Transform> avg_pos = decltype(avg_pos)(num_markers);

  void broadcast_frame(geometry_msgs::Transform transform, int id_num) {
    std::string str = std::to_string(id_num);
    std::string frame_id = "world_marker_"+str;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_link";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform=transform;
    br.sendTransform(transformStamped);
  }

  void tracker(geometry_msgs::TransformStamped msg){
    //Find frame in regard to world

    geometry_msgs::TransformStamped frame;
    bool transform_succes;
    ros::Time stamp = msg.header.stamp;
    //wait to make sure pose is in the buffer
    ros::Duration(0.20).sleep();
    try{
       frame = buffer_.lookupTransform( "world",msg.child_frame_id,stamp);
       transform_succes=true;
    }

    catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
         transform_succes=false;
   }
   if (transform_succes==true) {
     //Get frame id as int
     std::string frame_id =msg.child_frame_id;
     frame_id.erase(0,7);
     int id_num = std::stoi(frame_id);
     //set avg translation
     avg[id_num][0].x += frame.transform.translation.x;
     avg[id_num][0].y += frame.transform.translation.y;
     avg[id_num][0].z += frame.transform.translation.z;
     //set avg rotation
     double rx, ry, rz;
     tf2::Quaternion q(frame.transform.rotation.x,frame.transform.rotation.y,frame.transform.rotation.z,frame.transform.rotation.w);
     tf2::Matrix3x3 matrix(q);

     matrix.getRPY(rx,ry,rz);

     avg[id_num][1].x += rx;
     avg[id_num][1].y += ry;
     avg[id_num][1].z += rz;
     counter[id_num] += 1;
     if (counter[id_num]==seen) {


       //set avg trans
       float tx_avg=avg[id_num][0].x/seen;
       float ty_avg=avg[id_num][0].y/seen;
       float tz_avg=avg[id_num][0].z/seen;
       //set avg rotation
       float rx_avg=avg[id_num][1].x/seen;
       float ry_avg=avg[id_num][1].y/seen;
       float rz_avg=avg[id_num][1].z/seen;

       //Set avg pos

       avg_pos[id_num].translation.x = tx_avg;
       avg_pos[id_num].translation.y = ty_avg;
       avg_pos[id_num].translation.z = tz_avg;

       tf2::Quaternion Q;
       Q.setRPY(rx_avg,ry_avg,rz_avg);
       avg_pos[id_num].rotation.x = Q.x();
       avg_pos[id_num].rotation.y = Q.y();
       avg_pos[id_num].rotation.z = Q.z();
       avg_pos[id_num].rotation.w = Q.w();
       broadcast_frame(avg_pos[id_num],id_num);
       if (marker_found[id_num]!=true) {
         num_markers_found +=1;
       }
       marker_found[id_num]=true;
       //Reset
       reset_all(id_num);

     }
   }
}
  void reset_all(int id_num) {
    //Reset all
    counter[id_num]=0;
    avg[id_num][0].x=0;
    avg[id_num][0].y=0;
    avg[id_num][0].z=0;
    //set avg rotation
    avg[id_num][1].x=0;
    avg[id_num][1].y=0;
    avg[id_num][1].z=0;
  }

};

//The most helpfull sheet youll meet today :)
//http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "Commander");
  //Define class intace
  tf_tracker instance;

  ros::Subscriber sub = instance.nh.subscribe("data/vision_data", 1, &tf_tracker::tracker,&instance);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface setEndEffectorLink("TCP"); //NOtE: THIS CAN BE ANY FRAME ON THE ENDEFFECTOR
  move_group.setPoseReferenceFrame("marker");

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

    base_marker.setOrigin(tf2::Vector3(instance.avg_pos[0].translation.x, instance.avg_pos[0].translation.y,instance.avg_pos[0].translation.z));
    base_marker.setRotation(tf2::Quaternion(instance.avg_pos[0].rotation.x,instance.avg_pos[0].rotation.y,instance.avg_pos[0].rotation.z,instance.avg_pos[0].rotation.w));



    goal_position = base_marker*look;





    tf2::Transform corner[]={corner1, corner2, corner3, corner4};




    //Move close to marker
    bool success = false;

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
    } while(success=false && ros::ok());


    move_group.setGoalPositionTolerance(0.00001);
    //prepare for calibration
    for (int i = 1; i <= 2; i++) {
      ROS_INFO("Starting Calibration (Part %d of 2)",i);
      ros::Duration(2.5).sleep();
      instance.seen=1200;
      instance.marker_found[0]==false;
      instance.reset_all(0);

    }
    ROS_INFO("Calibrating...");
    do {
      ros::Duration(0.01).sleep();
    } while(instance.marker_found[0]==false && ros::ok());


/*    do {
      move_group.setPoseTarget(target_pose,"TCP");
      move_group.plan(go_to_marker);
      move_group.move();
      target_pose.position.z = instance.avg_pos[0].translation.z+0.0492;
      ROS_INFO("Done");
    } while(ros::ok());*/



    for (size_t i = 0; i < 4; i++) {

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
        move_group.plan(go_to_marker);
        move_group.move();
      //  base_marker.position.z = instance.avg_pos[0].translation.z;
        //base_marker.position.y = instance.avg_pos[0].translation.y;
        //base_marker.position.x = instance.avg_pos[0].translation.x;
        ROS_INFO("Corner 1");
      } while(ros::ok());

      ros::Duration(0.5).sleep();
  }




  ros::spin();
}
