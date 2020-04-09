#include <ros/ros.h>
#include <iostream>
#include <fstream>

//MSGs
#include <geometry_msgs/TransformStamped.h>
#include <vision_lego/TransformRPYStamped.h>

//Prossesing
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class data{
private:
  std::string file_path;
  std::string file_name;
  std::string separator;
  double sample_size;
  double counter;
protected:
  std::ofstream file;


public:
ros::NodeHandle nh;
void load_param()
{
  ros::param::param<std::string>("/file_path", file_path, "/home/peter/lego_ws/src/rob5_lego/zero_point_cal/markers/");
  ros::param::param<std::string>("/file_name", file_name, "csv_output.csv");
  ros::param::param<std::string>("/decimal_separator", separator, ";");
  ros::param::param<double>("/sample_size", sample_size, 1000);
  ROS_INFO("Read parameters");
}

void write_csv(float x,float y,float z,double R,double P,double Y) {

  //Test if the file is done already
  if (counter<=sample_size) {
    //Test if file is open otherwhice open
    if (file.is_open()!=true) {
      file.open(file_path+file_name);
      ROS_INFO("Writing file: %s",file_name.c_str());
    }
    //add data to line
    file << x << separator << y << separator << z << separator << R << separator << P << separator << Y <<std::endl;
    //add to counter
    counter++;
    //if we have the number of smaples specified close the file and say done
    if(counter=sample_size){
      file.close();
      ROS_INFO("File done");
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
};

int main(int argc, char **argv)
{
  //Ros init stuff:
  ros::init(argc, argv, "data_extractor");

  //Define class instance
  data extractor;

  //load in parameters
  extractor.load_param();

  //import data from marker pose.
  ros::Subscriber data_importer = extractor.nh.subscribe("data/vision_data", 30, &data::poseCallback,&extractor);

  ros::spin();
}
