#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>
#include <vision_lego/TransformRPYStamped.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/dictionary.hpp>

#include <cv_bridge/cv_bridge.h>

#include <vision_lego/GetMarker.h>

class markers{
private:
  // camera calibration, is found using http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  // and the topic /camera/color/came_info
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  double to_degree = 180/3.14159265359;
  int num_markers;
  int marker_bits_size; //X by X bits
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
  double marker_width;
protected:
  int cam_matrix_mode;
  int corner_refinement_method;

public:
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<vision_lego::TransformRPYStamped>("data/vision_data", 100);

  void load_param()
  {
    //load marker relevant information
    ros::param::param<int>("/num_markers", num_markers, 14);
    ros::param::param<int>("/marker_bits_size", marker_bits_size, 4);
    dictionary = cv::aruco::generateCustomDictionary(num_markers, marker_bits_size);
    
    //Load cornerRefinementMethod
    ros::param::param<int>("/corner_refinement_method", corner_refinement_method, 1);
    switch (corner_refinement_method) {
      case 0: detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
      case 1: detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
      case 2: detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
    }
    //Set camera matrix bades on highres or low res mode
    ros::param::param<int>("/cam_matrix_mode", cam_matrix_mode, 2);
    switch (cam_matrix_mode) {
      case 1: //high_res_default_matrix
              cameraMatrix = (cv::Mat_<double>(3,3) << 922.05810546875, 0.0, 633.1328125, 0.0, 922.3731689453125, 365.50604248046875, 0.0, 0.0, 1.0);
              distCoeffs= (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
      case 2: //low_res_deafault_matrix
              cameraMatrix = (cv::Mat_<double>(3,3) << 614.7054443359375, 0.0, 315.4218444824219, 0.0, 614.9154052734375, 243.67068481445312, 0.0, 0.0, 1.0);
              distCoeffs= (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
      case 3: //high_res_custom_matrix

      case 4: //low_res_custom_matrix
              cameraMatrix = (cv::Mat_<double>(3,3) << 610.751037, 0.0, 323.371090, 0.0, 610.304837,245.105395, 0.0, 0.0, 1.0);
              distCoeffs= (cv::Mat_<double>(1,5) << 0.112719, -0.238286, 0.003201, 0.003401, 0.0);
              ROS_WARN("low_res_custom_matrix in use");

    }

    ros::param::param<double>("/marker_width", marker_width, 0.0094);

  }

  void broadcast_frame(geometry_msgs::TransformStamped transformStamped, vision_lego::TransformRPYStamped vision_data) {
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transformStamped);
    pub.publish(vision_data);
  }

  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    cv::Mat image;
    //import image from msg
    try
    {
      //convert compressed image data to cv::Mat
      image = cv::imdecode(cv::Mat(msg->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
      //if that didnt work, display error in terminal
      ROS_ERROR("Could not convert to image!");
    }
    //Your code here
    cv::Mat imageCopy;
    image.copyTo(imageCopy);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(imageCopy, dictionary, corners, ids,detectorParams);
    // if at least one marker detected
    std::vector<cv::Vec3d> rvecs, tvecs;

    std_msgs::Header header=msg->header;
    if (ids.size() > 0){
      cv::aruco::estimatePoseSingleMarkers(corners, marker_width, cameraMatrix, distCoeffs, rvecs, tvecs);

      for (int i = 0; i < ids.size(); i++) {
        geometry_msgs::TransformStamped frame;
        //find the corect frame id
        std::string id_num= std::to_string(ids[i]);
        std::string frame_id = "marker_"+id_num;
        //add the camera header
        frame.header.stamp = msg->header.stamp;
        frame.header.frame_id = "camera_color_optical_frame";
        frame.child_frame_id = frame_id;
        frame.transform.translation.x = tvecs[i][0];
        frame.transform.translation.y = tvecs[i][1];
        frame.transform.translation.z = tvecs[i][2];
        //find Rotation as Quaternion
        float angle = sqrt((rvecs[i][0]*rvecs[i][0])
                            +(rvecs[i][1]*rvecs[i][1])
                            +(rvecs[i][2]*rvecs[i][2]));

        tf2::Vector3 rvec(rvecs[i][0],rvecs[i][1],rvecs[i][2]);
        //write as Quaternion
        tf2::Quaternion q;
        q.setRotation(rvec,angle);
        frame.transform.rotation.x = q.x();
        frame.transform.rotation.y = q.y();
        frame.transform.rotation.z = q.z();
        frame.transform.rotation.w = q.w();
        //calculate RPY from Quaterions
        vision_lego::TransformRPYStamped vision_data;
        vision_data.header=frame.header;
        vision_data.child_frame_id=frame.child_frame_id;
        vision_data.translation=frame.transform.translation;
        double Y, P, R;
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(Y, P, R);
        vision_data.orientation.Roll=R;
        vision_data.orientation.Pitch=P;
        vision_data.orientation.Yaw=Y;
        broadcast_frame(frame,vision_data);

      }
    }
  }

  bool service_get_marker(vision_lego::GetMarker::Request &req,vision_lego::GetMarker::Response &res)
	{
    std::string path = req.path;
    cv::Mat marker;
    for (int id_num = 0; id_num < num_markers; id_num++) {
      cv::aruco::drawMarker(dictionary, id_num, 200, marker, 1);
      //save marker is pdf
      std::string number = std::to_string(id_num);
      std::string img_type = ".png";
      std::string slash ="/";
      std::string full_path = path+slash+number+img_type;
      //Define params to save image
      std::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(9);
      try {
          cv::imwrite(full_path, marker, compression_params);
          ROS_INFO("Saved marker:%s",full_path.c_str());
          res.finished =true;
      }
      catch (cv::Exception& e) {
          ROS_ERROR("Error converting image to PNG format");
          res.finished = false;
      }
    }
    return true;
  }
};



int main(int argc, char **argv)
{
  //Ros init stuff:
  ros::init(argc, argv, "marker_detection");

  //Define class instance
  markers instance;
  ros::Duration(5).sleep();
  instance.load_param();
  //Setup publisher and subscriber

  ros::Subscriber sub = instance.nh.subscribe("/camera/color/image_raw/compressed", 30, &markers::imageCallback,&instance);
  ros::ServiceServer service = instance.nh.advertiseService("get_marker", &markers::service_get_marker,&instance);

  ros::spin();
}
