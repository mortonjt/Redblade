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

double maxHeight = 2; 
double tolerance = 0.7;
double sigSize = 100;//Anything below this isn't signficant


redblade_stereo::redblade_stereo(double r,double z, double w){
  groundHeight = z;
  viewingRadius = r;
  poleWidth = w;
}

redblade_stereo::~redblade_stereo(){}



//Filters out ground using a passthrough filter
void redblade_stereo::filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  int numFiltered = 0;
  //filtered->points.resize(cloud->width*cloud->height);
  //filtered->points.resize(cloud->width);
  for(size_t i = 0; i<cloud->points.size();++i){
    //if(cloud->points[i].y < -1*maxHeight and cloud->points[i].y > -1*groundHeight){
    if(cloud->points[i].y < groundHeight){
	// ROS_INFO("x %f, y %f, z %f",
	// 	 cloud->points[i].x,
	// 	 cloud->points[i].y,
	// 	 cloud->points[i].z);
	//filtered->points[numFiltered++] = cloud->points[i];
	filtered->points.push_back(cloud->points[i]);
      }
    }
    //filtered->points.resize(numFiltered);  
}



//Filters out ground using a passthrough filter
void redblade_stereo::filterBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				       pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  int numFiltered = 0;
  filtered->points.resize(cloud->width*cloud->height);

  for(size_t i = 0; i<cloud->points.size();++i){
    double distance = \
      sqrt(cloud->points[i].x*cloud->points[i].x+	\
	   cloud->points[i].y*cloud->points[i].y+	\
	   cloud->points[i].z*cloud->points[i].z);
    if( distance < viewingRadius){
      // ROS_INFO("x %f, y %f, z %f Distance %f",
      // 	       cloud->points[i].x,
      // 	       cloud->points[i].y,
      // 	       cloud->points[i].z,
      // 	       distance);
      //filtered->points.push_back(cloud->points[i]);
      filtered->points[numFiltered++] = cloud->points[i];
    }
  }
  filtered->points.resize(numFiltered);  
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

/*
  When point is publish, use a new orientation
  x
  ^
  |
  | 
  |
  ^---------->y
 */

void redblade_stereo::cloud2point(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
				  geometry_msgs::Point& point){
  double totalx=0,totaly=0;
  for(size_t i = 0; i<in->points.size();++i){
    totalx+= in->points[i].z;
    totaly+= in->points[i].x;
  }
  //Just average
  point.x = totalx/((double)in->points.size());
  point.y = totaly/((double)in->points.size());
  point.z = 0; //TODO: Do we want to put in a point for the pole?
}

//Finds the pole using the RANSAC algorithm
bool redblade_stereo::findPole(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
			       pcl::PointCloud<pcl::PointXYZ>::Ptr pole){
  ROS_INFO("Size of input cloud %d",in->points.size());
  Eigen::VectorXf coeff;
  coeff.resize(6);
  ransac(in,pole,coeff);
  if(coeff.size()==0){
    return false;
  }
  ROS_INFO("Model Coefficients (x:%f,y:%f,z:%f)+(dx:%f,dy:%f,dz:%f)",
	   coeff[0],coeff[1],coeff[2],coeff[3],coeff[4],coeff[5]);
  if(fabs(coeff[4])>tolerance){//Vertical line test
    if(pole->points.size()>sigSize){//significance test
      return true;}
  }
  return false;
}
