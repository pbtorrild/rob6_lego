#include <ros/ros.h>
#include <iostream>
#include <fstream>

//MSGs
#include <geometry_msgs/TransformStamped.h>

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

void poseCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
  //look up the xyz
  float x = msg->transform.translation.x;
  float y = msg->transform.translation.y;
  float z = msg->transform.translation.z;

  //calculate RPY
  double Y, P, R;
  tf2::Quaternion q(msg->transform.rotation.x,msg->transform.rotation.y,msg->transform.rotation.z,msg->transform.rotation.w);
  tf2::Matrix3x3 matrix(q);
  matrix.getRPY(Y, P, R);

  //Wtrite to file
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
  ros::Subscriber data_importer = extractor.nh.subscribe("tf/marker_msgs", 30, &data::poseCallback,&extractor);

  ros::spin();
}
