#include "redblade_laser.h"


ros::Publisher pub;
sensor_msgs::LaserScan currentScan;
geometry_msgs::Pose2D currentPose;
laser_geometry::LaserProjection projector_;
bool hasPoints;
bool hasPose;

void pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  currentPose = *pose_msg;
  hasPose = true;
  //ROS_INFO("Got pose");
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  currentScan = *scan_in;
  hasPoints = true;
}

void publish_loop(){
  sensor_msgs::PointCloud2 cloud;
  projector_.projectLaser(currentScan, cloud); 

}



int main(int argc, char** argv){
  ros::init(argc, argv, "redblade_laser");
  ros::NodeHandle nh;
  ros::NodeHandle n("~"); 
  std::string laser_namespace,pole_namespace,ekf_namespace;
  hasPoints = false;
  hasPose = false;


}
