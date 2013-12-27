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

double maxHeight = -2; //Crop everything above 2 m
double tolerance = 0.001;
double sigSize = 100;//Anything below this isn't signficant


redblade_stereo::redblade_stereo(int r,int z, int w){
  groundHeight = z;
  viewingRadius = r;
  poleWidth = w;
}

redblade_stereo::~redblade_stereo(){}



//Filters out ground using a passthrough filter
void redblade_stereo::filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  //filtered->points.resize(cloud->width);
  for(size_t i = 0; i<cloud->points.size();++i){
    if(cloud->points[i].y < -1*maxHeight and cloud->points[i].y > -1*groundHeight){
      filtered->push_back(cloud->points[i]);
    }
  }
}
//Filters out ground using a passthrough filter
void redblade_stereo::filterBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				       pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  for(size_t i = 0; i<cloud->points.size();++i){
    double distance = \
      sqrt(cloud->points[i].x*cloud->points[i].x+	\
	   cloud->points[i].y*cloud->points[i].y+	\
	   cloud->points[i].z*cloud->points[i].z);
    std::cout<<"Distance "<<distance<<std::endl;
    if( distance < viewingRadius){
      filtered->push_back(cloud->points[i]);
    }
  }
}
void redblade_stereo::ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
			     pcl::PointCloud<pcl::PointXYZ>::Ptr pole,
			     Eigen::VectorXf& coeff){
  std::vector<int> inliers;
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr 
    model(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(in));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_obj(model);
  ransac_obj.setDistanceThreshold(0.1);
  ransac_obj.computeModel();
  ransac_obj.getInliers(inliers);
  pcl::copyPointCloud<pcl::PointXYZ>(*in,inliers,*pole);
  ransac_obj.getModelCoefficients(coeff);
}
//Finds the pole using the RANSAC algorithm
bool redblade_stereo::findPole(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
			       pcl::PointCloud<pcl::PointXYZ>::Ptr pole){
  Eigen::VectorXf coeff;
  coeff.resize(6);
  ransac(in,pole,coeff);
  if(coeff.size()==0){
    return false;
  }
  // if(abs(coeff[3])<tolerance and abs(coeff[5])<tolerance){//Vertical line test
  if(pole->points.size()>sigSize){//significance test
    return true;}
  //}
  return false;
}
