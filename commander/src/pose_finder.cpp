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
  //Make a vector to take avg so that:
  //avg[id][0].x = tx
  //avg[id][1].x = rx
  std::vector<cv::Point3f> Point;
  std::vector<std::vector<cv::Point3f>> avg; //Create vector of ids containg a vector of size 4 each containing a cv point 3f
  std::vector<int> counter;

  //Vector containg the avg_pos of the markers
  std::vector<geometry_msgs::Transform> avg_pos;
  std::vector<bool> marker_found;
protected:

public:

  bool num_markers_found;
  tf_tracker():
  tf2_(buffer_),  target_frame_("base_link")
  {

  }
  ros::NodeHandle nh;
  ros::Publisher pub_latest = nh.advertise<geometry_msgs::TransformStamped>("data/markers/latest_transform", 5);
  ros::Publisher pub_avg = nh.advertise<geometry_msgs::TransformStamped>("data/markers/running_avg", 5);

  void load_param() {
    ros::param::param<int>("/num_markers", num_markers, 14);
    ros::param::param<int>("/avg_gate", avg_gate, 300);
    declare_values();
    ROS_INFO("Parameters are now set");
  }

  void declare_values() {
    //Make a vector to take avg so that:
    //avg[id][0].x = tx
    //avg[id][1].x = rx
    Point = decltype(Point)(2);
    avg = decltype(avg)(num_markers,Point); //Create vector of ids containg a vector of size 4 each containing a cv point 3f
    counter =decltype(counter)(num_markers);

    //Vector containg the avg_pos of the markers
    avg_pos = decltype(avg_pos)(num_markers);
    marker_found =decltype(marker_found)(num_markers);
  }

  void publish_latest(geometry_msgs::TransformStamped transform_in) {
    pub_latest.publish(transform_in);
  }
  void publish_avg(geometry_msgs::Transform transform, int id_num) {
    std::string str = std::to_string(id_num);
    std::string frame_id = "avg_marker_"+str;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform=transform;
    pub_avg.publish(transformStamped);
  }

  void broadcast_frame(geometry_msgs::Transform transform, int id_num) {
    std::string str = std::to_string(id_num);
    std::string frame_id = "avg_marker_"+str;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = frame_id;
    transformStamped.transform=transform;
    br.sendTransform(transformStamped);
  }


  void tracker(vision_lego::TransformRPYStamped msg){
    //Find frame in regard to world

    geometry_msgs::TransformStamped frame;
    bool transform_succes;
    //wait to make sure pose is in the buffer
    ros::Duration(0.20).sleep();
    try{
       frame = buffer_.lookupTransform( "base_link",msg.child_frame_id,msg.header.stamp);
       transform_succes=true;
    }

    catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
         transform_succes=false;
   }
   if (transform_succes==true) {
     //Send to latest Publisher
     publish_latest(frame);
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

     if (counter[id_num]==avg_gate) {

       //set avg trans
       float tx_avg=avg[id_num][0].x/avg_gate;
       float ty_avg=avg[id_num][0].y/avg_gate;
       float tz_avg=avg[id_num][0].z/avg_gate;
       //set avg rotation
       float rx_avg=avg[id_num][1].x/avg_gate;
       float ry_avg=avg[id_num][1].y/avg_gate;
       float rz_avg=avg[id_num][1].z/avg_gate;

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
       publish_avg(avg_pos[id_num],id_num);
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "Transfrom_transfromer");
  //Define class intace
  tf_tracker instance;
  instance.load_param();
  ros::Duration(3).sleep();
  ros::Subscriber sub = instance.nh.subscribe("data/vision_data", 1, &tf_tracker::tracker,&instance);


  ros::spin();
}
