#include <vector>
#include <deque>
#include <stdio.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

class redblade_laser{
 public:
  double laserOffset;
  std::string surveyFile;
  std::vector<double> x;
  std::vector<double> y;
  std::deque< pcl::PointCloud<pcl::PointXYZ>::Ptr > queue;
  int maxSize;  //Number of scan frames stored in queue
  
  /*offset: length displacment of laser from the center of the robot*/
  redblade_laser(std::string surveyFile,double laserOffset,int queueSize);
  
  /*Add laser scan to queue*/
  void addScan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /*Retrieve point clouds from queue*/
  void getClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /*Check if points are within the survey field*/
  bool inBounds(double x, double y);
  
  /*Transforms laser coordinates into ENU coordinates*/
  void transformLaser2ENU(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /*Filters everything outside of survey field*/
  void filterBackground(geometry_msgs::Pose2D pose,
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered);

  
};
