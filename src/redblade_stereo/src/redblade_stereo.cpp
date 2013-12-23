#include "redblade_stereo.h"



/*
  Orientation
z                               Basically, y is upside down
^                               and z is pointing into the page
 \
  \
   \
    *----------------> x
    |
    |
    |
    |
    |
    |
    v
    y
 */

redblade_stereo::redblade_stereo(int z){
  groundHeight = z;
  maxHeight = -3;  //Crop everything above 3 m
}

redblade_stereo::~redblade_stereo(){}



//Filters out ground using a passthrough filter
void redblade_stereo::filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr points){
  // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(points);
  // pass.setFilterFieldName("y");
  // pass.setFilterLimits(-maxHeight,-1*groundHeight);
  // pass.filter(*filtered);
  // points = filtered;
}
  
//Finds the pole using the RANSAC algorithm
void redblade_stereo::findPole(pcl::PointCloud<pcl::PointXYZ>& points){
  
}
