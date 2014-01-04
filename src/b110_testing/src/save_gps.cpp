#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <cmath>
#include <queue>
#include <iostream>
#include <fstream>

std::string filename;

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg){
  
  std::string specificFilename;

  switch(fix_msg.header.frame_id){
  case "B110_1":
    specificFilename = filename + "_B110_1.csv";
    break;
  case "B110_2":
    specificFilename = filename + "_B110_2.csv";
    break;
  case "HiperLitePlus":
    specificFilename = filename + "_HiperLitePlus.csv";
    break;
  default:
    ROS_WARN("INVALID REFERENCE FRAME.");
    return;
  }
  
  std::fstream fs(specificFilename,ios+base::openmode mode = ios_base::out | ios_base::app);
  
  fs << fix_msg.latitude << "," << fix_msg.longitude << "," << fix_msg.altitude << "," //lat, long, alt 
     << fix_msg.header.stamp.sec << "," << fix_msg.header.stamp.nsec << ","            //time
     << fix_msg.position_covariance[0] << "," << fix_msg.position_covariance[1] << "," //covariance
     << fix_msg.position_covariance[2] << "," << fix_msg.position_covariance[3] << "," //covariance
     << fix_msg.position_covariance[4] << "," << fix_msg.position_covariance[5] << "," //covariance
     << fix_msg.position_covariance[6] << "," << fix_msg.position_covariance[7] << "," //covariance
     << fix_msg.position_covariance[8] << endl;                                        //covariance

}

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "gps_save_node");
  ros::NodeHandle n;//global namespace
  ros::NodeHandle nh("~");//local namespace, used for params

  // //Start spinner so that callbacks happen in a seperate thread
  // ros::AsyncSpinner spinner(1);//1 threads
  // spinner.start();

  nh.param("filename", filename, "");

  //Subscribe to GPS topic
  ros::Subscriber fix_sub = n.subscribe("/fix", 0, fixCallback);//infinite queue. I don't want to miss any data points

  ros::spin();

  return 0;
}
