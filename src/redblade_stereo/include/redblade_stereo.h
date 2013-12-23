#include <image_transport/image_transport.h>

#include <stdlib.h>
#include <stdio.h>

#include <stdint.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
//#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/octree/octree.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

class redblade_stereo{
 public:
  double groundHeight; //maximum height of ground
  double maxHeight;


  redblade_stereo(int z);
  ~redblade_stereo();
  //Filters out ground using a passthrough filter
  void filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered);
  //Filters out everything outside of 2 m
  void filterBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered);
  //Finds the pole using the RANSAC algorithm
  void ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
	      pcl::PointCloud<pcl::PointXYZ>::Ptr pole);
  void findPole(pcl::PointCloud<pcl::PointXYZ>& points);
};
