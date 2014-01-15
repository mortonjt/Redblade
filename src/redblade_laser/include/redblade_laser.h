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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <laser_geometry/laser_geometry.h>

#include <assert.h>

#define zoneWidth   4    //Width of plowing zone
#define fieldWidth  1    //Width of snow field
#define zoneLength  18   //Length of plowing zone
#define fieldLength 10   //Length of snow field

class redblade_laser{
 public:
  double laserOffset;
  std::string surveyFile;
  std::vector<double> x;
  std::vector<double> y;
  std::deque< pcl::PointCloud<pcl::PointXYZ>::Ptr > queue;
  double fieldAngle;

  int maxSize;          //Number of scan frames stored in queue
  bool searchSnowField; //Indicates whether to search inside of the snow field or not
  /*offset: length displacment of laser from the center of the robot*/
  redblade_laser(std::string surveyFile,
		 double laserOffset,int queueSize);
  redblade_laser(std::string surveyFile,
		 bool searchSnowField,double laserOffset,int queueSize);
  
  bool saturated();//Tests to see if the queue is full

  void scan2cloud(sensor_msgs::LaserScan::Ptr,
		  pcl::PointCloud<pcl::PointXYZ>::Ptr);
  /*Add laser scan to queue*/
  void addScan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /*Retrieve point clouds from queue*/
  void getClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  void rotate(double& x, double& y);
  /*Check if points are within the survey field of interest*/
  bool inBounds(double x, double y);
  
  /*Check if transformed points are inside of the snow field*/
  bool inSnowField(double transformedX, double transformedY);
  
  /*Transforms laser coordinates into ENU coordinates*/
  void transformLaser2ENU(geometry_msgs::Pose2D& currentPose,
			  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /*Filters everything outside of survey field*/
  void filterBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered);

  /*Performs Euclidean clustering to find pole*/
  void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	       pcl::PointCloud<pcl::PointXYZ>::Ptr cluster,
	       double tolerance);
  
  /*Finds the pole*/
  bool findPole(geometry_msgs::Point& point,double tolerance);

  /*Condense point cloud into a single point*/
  void cloud2point(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
		   geometry_msgs::Point& point);


};
