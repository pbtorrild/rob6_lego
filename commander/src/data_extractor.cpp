#include <ros/ros.h>
#include <iostream>
#include <fstream>

//MSGs
#include <geometry_msgs/Pose.h>
#include <commander/PoseTCP.h>


//Prossesing
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class data{
private:
  std::string file_path;
  std::string file_name;
  std::string separator;
  int32_t sample_size;
  int32_t counter;
  int procent;
  ros::Time timer;
protected:
  std::ofstream file;


public:
ros::NodeHandle nh;

void load_param()
{
  ros::param::param<std::string>("/file_path", file_path, "/home/peter/");
  ros::param::param<std::string>("/file_name", file_name, "csv_output.csv");
  ros::param::param<std::string>("/separator", separator, ";");
  ros::param::param<int32_t>("/sample_size", sample_size, 100);
  ROS_INFO("Read parameters!");
  ROS_INFO("File Path: %s",file_path.c_str());
  ROS_INFO("File Name: %s",file_name.c_str());
  ROS_INFO("Sample Size: %d",sample_size);
}

void write_csv(float x,float y,float z,double R,double P,double Y, double time) {

  //Test if the file is done already
  if (counter<=sample_size) {
    //Test if file is open otherwhice open
    if (file.is_open()!=true) {
      file.open(file_path+file_name);
      ROS_INFO("Writing file: %s",file_name.c_str());
      file << "x_error" << separator << "y_error" << separator << "z_error" << separator << "R_error" << separator << "P_error" << separator << "Y_error" << separator << "time" <<std::endl;

    }
    //add data to line
    file << x << separator << y << separator << z << separator << R << separator << P << separator << Y << separator << time << std::endl;
    //add to counter

    if (counter%(sample_size/20)==0) {
      ROS_INFO("%d %% done with the file",procent*5);
      procent++;
    }
    counter++;
    //if we have the number of smaples specified close the file and say done
    if(counter==sample_size){
      file.close();
      ROS_INFO("File done");
      ros::shutdown();
    }
  }
}

  void poseCallback(commander::PoseTCP msg)
  {
    //TCP RPY
    tf2::Quaternion q_tcp(msg.tcp_location.orientation.x,msg.tcp_location.orientation.y,msg.tcp_location.orientation.z,msg.tcp_location.orientation.w);
    double tcp_R,tcp_P,tcp_Y;
    tf2::Matrix3x3 tcp_matrix(q_tcp);
    tcp_matrix.getRPY(tcp_R, tcp_P, tcp_Y);

    //Goal RPY
    tf2::Quaternion q_goal(msg.goal_location.orientation.x,msg.goal_location.orientation.y,msg.goal_location.orientation.z,msg.goal_location.orientation.w);
    double goal_R,goal_P,goal_Y;
    tf2::Matrix3x3 goal_matrix(q_goal);
    goal_matrix.getRPY(goal_R, goal_P, goal_Y);

    //Calculate errors
    double x,y,z,R,P,Y;
    x = msg.goal_location.position.x - msg.tcp_location.position.x;
    y = msg.goal_location.position.y - msg.tcp_location.position.y;
    z = msg.goal_location.position.z - msg.tcp_location.position.z;
    R = goal_R - tcp_R;
    P = goal_P - tcp_P;
    R = goal_Y - tcp_Y;

    double time =  ros::Time::now().toSec()-timer.toSec();
    timer = ros::Time::now();
    //Write to file
    write_csv(x,y,z,R,P,Y,time);

  }

};

int main(int argc, char **argv)
{
  //Ros init stuff:
  ros::init(argc, argv, "pose_data_extractor");

  //Define class instance
  data extractor;

  //load in parameters
  extractor.load_param();

  //import data from marker pose.
  ros::Subscriber data_importer = extractor.nh.subscribe("data/tcp_location", 300, &data::poseCallback,&extractor);

  ros::spin();
}
