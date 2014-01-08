#include "redblade_stereo.h"

/*
  Camera Orientation                            Robot Orientation	       	   
z                                                             
^                                                             
 \				     		       	   
  \				     	       	                  z
   \				     			          ^
    *----------------> x	                            x     |
    |				                 	     \    |  			   
    |				                 	      \   |  			   
    |				                 	       \  |  			   
    |				                 	        \ |  			   
    |				                                 \|  			   
    |				                   y<-------------*  			   
    v				                  			   
    y				                                       
				    	      			       	   
Basically, y is upside down    	    	      X is pointing into the page   
and z is pointing into the page	    	      
 */

//Constants
double maxHeight = 3; //Filters out everything above a reasonable height (probably can use height of pole)
double verticalTolerance = 0.7; //Mininum vertical slope for RANSAC
double sigSize = 250;//Anything below this isn't signficant
//double viewingWidth = 2.0;  //Filters everything outside of 2m of the robot's horizontal view
redblade_stereo::redblade_stereo(double viewingRadius, 
				 double viewingWidth,
				 double groundHeight, 
				 double poleWidth,
				 double cameraHeight,
				 double cameraLengthOffset){
  this->groundHeight = groundHeight;
  this->viewingRadius = viewingRadius;
  this->viewingWidth = viewingWidth;
  this->poleWidth = poleWidth;
  this->cameraHeight = cameraHeight;
  this->cameraLengthOffset = cameraLengthOffset;
}

redblade_stereo::~redblade_stereo(){}


void redblade_stereo::transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
 for(size_t i = 0; i<cloud->points.size();++i){
   double height = cloud->points[i].y;
   cloud->points[i].y = -1*cloud->points[i].x;
   cloud->points[i].x = cloud->points[i].z+this->cameraLengthOffset;
   cloud->points[i].z = this->cameraHeight-height;
 }
}

//Filters out ground using a passthrough filter
void redblade_stereo::filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  int numFiltered = 0;
  //filtered->points.resize(cloud->width*cloud->height);
  //filtered->points.resize(cloud->width);
  for(size_t i = 0; i<cloud->points.size();++i){
    if(cloud->points[i].z > groundHeight){// and cloud->points[i].y < maxHeight){
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
  //filtered->points.resize(cloud->width*cloud->height);

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
      if( fabs(cloud->points[i].y)<viewingWidth){
	//filtered->points[numFiltered++] = cloud->points[i];
	filtered->points.push_back(cloud->points[i]);
      }
    }
  }
  //filtered->points.resize(numFiltered);  
}

int redblade_stereo::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in,double tolerance){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(in);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(in);
  ec.extract(cluster_indices);
  return cluster_indices.size();
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
  std::vector<double> x,y;
  int n = in->points.size();
  for(size_t i = 0; i<in->points.size();++i){
    x.push_back(in->points[i].x);
    y.push_back(in->points[i].y);
  }
  std::sort(x.begin(),x.end());
  std::sort(y.begin(),y.end());
  point.x = x[n/2];
  point.y = x[n/2];
  point.z = 0;
  // double totalx=0,totaly=0;
  // for(size_t i = 0; i<in->points.size();++i){
  //   totalx+= in->points[i].x;
  //   totaly+= in->points[i].y;
  // }
  // //Just average
  // point.x = totalx/((double)in->points.size());
  // point.y = totaly/((double)in->points.size());
  // point.z = 0; //TODO: Do we want to put in a z-coordinate for the pole?
}

//Finds the pole using the RANSAC algorithm
bool redblade_stereo::findPole(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
			       pcl::PointCloud<pcl::PointXYZ>::Ptr pole){
  //ROS_INFO("Size of input cloud %d",in->points.size());
  Eigen::VectorXf coeff;
  coeff.resize(6);
  ransac(in,pole,coeff);
  if(coeff.size()==0){
    return false;
  }
  
  // ROS_INFO("Model Coefficients (x:%f,y:%f,z:%f)+(dx:%f,dy:%f,dz:%f)",
  // 	   coeff[0],coeff[1],coeff[2],coeff[3],coeff[4],coeff[5]);
  if(fabs(coeff[5])>verticalTolerance){     //Vertical line test
    if(pole->points.size()>sigSize){//Significance test
      // int clusters = cluster(pole,0.01); //1 cm
      // if(clusters==1){              //Test to see if the line is one contiguous segment
      // 	ROS_INFO("Size of line %d, Number of clusters %d",
      // 		 pole->points.size(),clusters);
      // ROS_INFO("Model Coefficients (x:%f,y:%f,z:%f)+(dx:%f,dy:%f,dz:%f)",
      // 	       coeff[0],coeff[1],coeff[2],coeff[3],coeff[4],coeff[5]);
      return true;}}
  //}
  return false;
}
