#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <string>
#include <fstream>

std::string filename;
const std::string empty = "";

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg){
  
  std::string specificFilename;
  //sensor_msgs::NavSatFix fix_msg(*nextSample);

  // std::float latitude = fix_msg->latitude;
  // std::float latitude = fix_msg->latitude;
  // std::float latitude = fix_msg->latitude;
  // std::float latitude = fix_msg->latitude;
  // std::float latitude = fix_msg->latitude;
  // std::float latitude = fix_msg->latitude;
  // std::float latitude = fix_msg->latitude;

  if(fix_msg->header.frame_id == "B110_1")
    specificFilename = filename + "_B110_1.csv";
  else if(fix_msg->header.frame_id == "B110_2")
    specificFilename = filename + "_B110_2.csv";
  else if(fix_msg->header.frame_id == "HiperLitePlus")
    specificFilename = filename + "_HiperLitePlus.csv";
  else{
    ROS_WARN("INVALID REFERENCE FRAME.");
    return;
  }
  
  std::fstream fs(specificFilename.c_str(), std::fstream::out | std::fstream::app);
  
  fs << fix_msg->latitude << "," << fix_msg->longitude << "," << fix_msg->altitude << "," //lat, long, alt 
     << fix_msg->header.stamp.sec << "," << fix_msg->header.stamp.nsec << ","            //time
     << fix_msg->position_covariance[0] << "," << fix_msg->position_covariance[1] << "," //covariance
     << fix_msg->position_covariance[2] << "," << fix_msg->position_covariance[3] << "," //covariance
     << fix_msg->position_covariance[4] << "," << fix_msg->position_covariance[5] << "," //covariance
     << fix_msg->position_covariance[6] << "," << fix_msg->position_covariance[7] << "," //covariance
     << fix_msg->position_covariance[8] << std::endl;                                   //covariance

  fs.close();

}

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "gps_record_node");
  ros::NodeHandle n;//global namespace
  ros::NodeHandle nh("~");//local namespace, used for params

  // //Start spinner so that callbacks happen in a seperate thread
  // ros::AsyncSpinner spinner(1);//1 threads
  // spinner.start();

  // nh.param("filename", filename, empty);

  //Subscribe to GPS topic
  ros::Subscriber fix_sub = n.subscribe("/fix", 0, fixCallback);//infinite queue. I don't want to miss any data points

  ros::spin();

  return 0;
}
