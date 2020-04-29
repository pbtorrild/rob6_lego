#include <ros/ros.h>
#include <iostream>
#include <fstream>

//MSGs
#include <geometry_msgs/TransformStamped.h>
#include <vision_lego/TransformRPYStamped.h>
#include <vision_lego/MarkerFound.h>


//Prossesing
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>

class data{
private:
  std::string file_path;
  std::string file_name;
  std::string separator;
  int32_t sample_size;
  int32_t counter;

  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  std::ofstream file;
protected:



public:
ros::NodeHandle nh;
//the frame tracker
data():
  tf2_(buffer_),  target_frame_("Base")
  {

  }
void load_param()
{

  ros::param::param<std::string>("/file_path", file_path, "/home/peter/lego_ws/src/rob6_lego/vision_lego/markers/");
  ros::param::param<std::string>("/file_name", file_name, "csv_output.csv");
  ros::param::param<std::string>("/separator", separator, ";");
  ros::param::param<int32_t>("/sample_size", sample_size, 1000);
  ROS_INFO("Read parameters!");
  ROS_INFO("File Path: %s",file_path.c_str());
  ROS_INFO("File Name: %s",file_name.c_str());
  ROS_INFO("Sample Size: %d",sample_size);
}

void write_csv(float x,float y,float z,double R,double P,double Y) {

  //Test if the file is done already
  if (counter<=sample_size) {
    //Test if file is open otherwhice open
    if (file.is_open()!=true) {
      file.open(file_path+file_name);
      ROS_INFO("Writing file: %s",file_name.c_str());
      file << "x" << separator << "y" << separator << "z" << separator << "R" << separator << "P" << separator << "Y" <<std::endl;

    }
    //add data to line
    file << x << separator << y << separator << z << separator << R << separator << P << separator << Y <<std::endl;
    //add to counter
    counter++;
    //if we have the number of smaples specified close the file and say done
    if(counter==sample_size){
      file.close();
      ROS_INFO("File done");
      ros::shutdown();
    }
  }
}

void write_bool_csv(bool marker_found, std::string marker_id) {
  //Test if the file is done already

  if (counter<=sample_size) {
    //Test if file is open otherwhice open
    if (file.is_open()!=true) {
      file.open(file_path+file_name);
      ROS_INFO("Writing file: %s",file_name.c_str());
      file << "marker_found" << separator << "marker_id" << std::endl;

    }
    //add data to line
    file << marker_found << separator << marker_id << std::endl;
    //add to counter
    counter++;
    //if we have the number of smaples specified close the file and say done
    if(counter==sample_size){
      file.close();
      ROS_INFO("File done");
      ros::shutdown();
    }
  }
}

void poseCallback(const vision_lego::TransformRPYStampedConstPtr& msg)
{
  //look up the xyz
  float x = msg->translation.x;
  float y = msg->translation.y;
  float z = msg->translation.z;

  //lookup RPY
  double R = msg->orientation.Roll;
  double P = msg->orientation.Pitch;
  double Y = msg->orientation.Yaw;

  //Write to file
  write_csv(x,y,z,R,P,Y);
}
void poseTransformCallback(const vision_lego::TransformRPYStampedConstPtr& msg) {
  //wait to make sure pose is in the buffer
    ros::Duration(0.20).sleep();
    geometry_msgs::TransformStamped look_up;
    bool transform_succes;
    ros::Time stamp = msg->header.stamp;
    //wait to make sure pose is in the buffer
    ros::Duration(0.20).sleep();
    try{
       look_up = buffer_.lookupTransform( "Base",msg->child_frame_id,stamp);
       transform_succes=true;
    }
    catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
         transform_succes=false;
    }

    //In order to simply fy i am only gonna look up the Z kooridiante
    //because this is the oinly value we can vertify with the given setup.
    //look up the xy from the msg
    float x = msg->translation.x;
    float y = msg->translation.y;
    //We look up the translation here from the
    float z = look_up.transform.translation.z;

    //lookup RPY
    double R = msg->orientation.Roll;
    double P = msg->orientation.Pitch;
    double Y = msg->orientation.Yaw;

    //Write to file
    write_csv(x,y,z,R,P,Y);
}

void marker_foundCallback(const vision_lego::MarkerFoundConstPtr& msg)
{
  write_bool_csv(msg->marker_found,msg->marker_id);
}

};




int main(int argc, char **argv)
{
  //Ros init stuff:
  ros::init(argc, argv, "data_extractor");

  //Define class instance
  data extractor;

  //load in parameters
  extractor.load_param();

  //import data from marker pose
  ros::Subscriber data_importer = extractor.nh.subscribe("data/vision_data", 300, &data::poseCallback,&extractor);
  ros::Subscriber data_importer2 = extractor.nh.subscribe("data/marker_found", 300, &data::marker_foundCallback,&extractor);



  ros::spin();
}
