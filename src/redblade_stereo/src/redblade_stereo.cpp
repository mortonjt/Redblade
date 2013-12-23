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
void redblade_stereo::filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  //filtered->points.resize(cloud->width);
  for(size_t i = 0; i<cloud->points.size();++i){
    if(cloud->points[i].y<-maxHeight and -1*groundHeight){
      filtered->push_back(cloud->points[i]);
    }
  }
  
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(points);
  // pass.setFilterFieldName("y");
  // pass.setFilterLimits(-maxHeight,-1*groundHeight);
  // pass.filter(*filtered);
}
void redblade_stereo::ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
			     pcl::PointCloud<pcl::PointXYZ>::Ptr pole){
  std::vector<int> inliers;
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr 
    model(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(in));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_obj(model);
  ransac_obj.setDistanceThreshold(0.05);
  ransac_obj.computeModel();
  ransac_obj.getInliers(inliers);
  pcl::copyPointCloud<pcl::PointXYZ>(*in,inliers,*pole);
}
  
//Finds the pole using the RANSAC algorithm
void redblade_stereo::findPole(pcl::PointCloud<pcl::PointXYZ>& points){
  
}
