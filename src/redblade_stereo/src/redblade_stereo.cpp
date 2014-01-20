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
double sigSize = 100;//Anything below this isn't signficant
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
  this->towerWidth = towerWidth;
  this->cameraHeight = cameraHeight;
  this->cameraLengthOffset = cameraLengthOffset;
  this->surveyFile="~";
}

redblade_stereo::redblade_stereo(std::string surveyFile,
				 bool tripleI,
				 bool searchSnowField,
				 double groundHeight, 
				 double poleWidth,
				 double cameraHeight,
				 double cameraLengthOffset){
  this->surveyFile = surveyFile;
  this->groundHeight = groundHeight;
  this->poleWidth = poleWidth;
  this->towerWidth = towerWidth;
  this->cameraHeight = cameraHeight;
  this->cameraLengthOffset = cameraLengthOffset;
  
  this->x.resize(4);
  this->y.resize(4);
  std::ifstream h((char*)surveyFile.c_str());
  h>>x[0]>>y[0]
   >>x[1]>>y[1];
  h.close();
  /*Make sure that the length of the field is within 1 cm of expected*/
  //assert( fabs((((x[0]-x[1])*(x[0]-x[1]) + (y[0]-y[1])*(y[0]-y[1]))) - zoneLength*zoneLength)<0.5);
  if(fabs((sqrt((x[0]-x[1])*(x[0]-x[1]) + (y[0]-y[1])*(y[0]-y[1]))) - zoneLength)>0.5){
    ROS_WARN("Double check survey - Measured Length:%lf Actual Length:%lf",
	     sqrt((x[0]-x[1])*(x[0]-x[1]) + (y[0]-y[1])*(y[0]-y[1])),zoneLength);
  }
  fieldAngle = -atan2(y[1],x[1]);

}

redblade_stereo::redblade_stereo(std::string surveyFile,
				 double groundHeight, 
				 double poleWidth,
				 double cameraHeight,
				 double cameraLengthOffset){
  this->surveyFile = surveyFile;
  this->groundHeight = groundHeight;
  this->poleWidth = poleWidth;
  this->towerWidth = towerWidth;
  this->cameraHeight = cameraHeight;
  this->cameraLengthOffset = cameraLengthOffset;
  
  this->x.resize(4);
  this->y.resize(4);
  std::ifstream h((char*)surveyFile.c_str());
  h>>x[0]>>y[0]
   >>x[1]>>y[1];
  h.close();
  /*Make sure that the length of the field is within 1 cm of expected*/
  //assert( fabs((((x[0]-x[1])*(x[0]-x[1]) + (y[0]-y[1])*(y[0]-y[1]))) - zoneLength*zoneLength)<0.5);
  if(fabs((sqrt((x[0]-x[1])*(x[0]-x[1]) + (y[0]-y[1])*(y[0]-y[1]))) - zoneLength)>0.5){
    ROS_WARN("Double check survey - Measured Length:%lf Actual Length:%lf",
	     sqrt((x[0]-x[1])*(x[0]-x[1]) + (y[0]-y[1])*(y[0]-y[1])),zoneLength);
  }
  fieldAngle = -atan2(y[1],x[1]);

}

bool redblade_stereo::inSnowField(double transformedX, double transformedY){
    if(tripleI){
      double width = 3;        //Width of snowfield
      double space = 2;      //Distance between zone and snow field
      double garageLength = 3; //Length of garage
      double plowedLength = 2; //Length of plowed snow zone
      return (transformedX>=garageLength and 
	      transformedX<=(zoneLength-plowedLength) and
	      transformedY>=space and
	      transformedY<=space+width);
    }else{
      double width = 1;        //Width of snowfield
      double space = 1.5;      //Distance between zone and snow field
      double garageLength = 3; //Length of garage
      double plowedLength = 2; //Length of plowed snow zone
      return (transformedX>=garageLength and 
	      transformedX<=(zoneLength-plowedLength) and
	      transformedY>=space and
	      transformedY<=space+width);
    }
}

void redblade_stereo::rotate(double& x, double& y){
  double transformedX = x*cos(this->fieldAngle)-y*sin(this->fieldAngle);
  double transformedY = x*sin(this->fieldAngle)+y*cos(this->fieldAngle);
  x = transformedX;
  y = transformedY;
}
bool redblade_stereo::inBounds(double x, double y){
  double transformedX = x;
  double transformedY = y;
  this->rotate(transformedX,transformedY);
  if(searchSnowField){
    return this->inSnowField(transformedX,transformedY);
  }else{
    if(tripleI){
      return ((not this->inSnowField(transformedX,transformedY)) and 
	      transformedX>=0 and transformedX<=zoneLength and transformedY>=0 and transformedY<=tripleIZoneWidth);
    }else{
      return ((not this->inSnowField(transformedX,transformedY)) and 
	      transformedX>=0 and transformedX<=zoneLength and transformedY>=0 and transformedY<=singleIZoneWidth);

    }
  }
}

redblade_stereo::~redblade_stereo(){}


void redblade_stereo::transformStereo2Robot(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
 for(size_t i = 0; i<cloud->points.size();++i){
   double height = cloud->points[i].y;
   cloud->points[i].y = -1*cloud->points[i].x;
   cloud->points[i].x = cloud->points[i].z+this->cameraLengthOffset;
   cloud->points[i].z = this->cameraHeight-height;
 }
}

void redblade_stereo::transformStereo2ENU(geometry_msgs::Pose2D& currentPose,
					  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  double x0      = currentPose.x;
  double y0      = currentPose.y;
  double theta   = currentPose.theta;
  for(size_t i = 0; i<cloud->points.size();++i){
    double height = cloud->points[i].y;
    double yc = -1*cloud->points[i].x;
    double xc = cloud->points[i].z+this->cameraLengthOffset;
    double zc = this->cameraHeight-height;
    cloud->points[i].x = xc*cos(theta)-yc*sin(theta)+x0;
    cloud->points[i].y = xc*sin(theta)+yc*cos(theta)+y0;
    cloud->points[i].z = zc;   
  }
}


void redblade_stereo::transformRobot2ENU(geometry_msgs::Pose2D& currentPose,
					 geometry_msgs::Point& localPolePoint,
					 geometry_msgs::Point& enuPolePoint){
  double xc      = localPolePoint.x;
  double yc      = localPolePoint.y;
  double theta   = currentPose.theta;
  double x0      = currentPose.x;
  double y0      = currentPose.y;
  enuPolePoint.x = xc*cos(theta)-yc*sin(theta)+x0;
  enuPolePoint.y = xc*sin(theta)+yc*cos(theta)+y0;
  enuPolePoint.z = 0;

}

//Filters out ground using a passthrough filter
void redblade_stereo::filterGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  for(size_t i = 0; i<cloud->points.size();++i){
    if(cloud->points[i].z > groundHeight){// and cloud->points[i].y < maxHeight){
	filtered->points.push_back(cloud->points[i]);
    }
  }
}


//Filters out ground using a passthrough filter
void redblade_stereo::filterBackground(geometry_msgs::Pose2D pose,
				       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				       pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  int numFiltered = 0;
  for(size_t i = 0; i<cloud->points.size();++i){
    // geometry_msgs::Point local,enu;
    // local.x = cloud->points[i].x;
    // local.y = cloud->points[i].y;
    // this->transformRobot2ENU(pose,local,enu);
    // if(this->inBounds(enu.x,enu.y)){
    //   filtered->points.push_back(cloud->points[i]);
    // }
    // ROS_INFO("x:%lf y:%lf z:%lf",
    // 	     cloud->points[i].x,
    // 	     cloud->points[i].y,
    // 	     cloud->points[i].z);
    if(this->inBounds(cloud->points[i].x,
		      cloud->points[i].y)){
      // ROS_INFO("In bounds x:%lf y:%lf z:%lf",
      // 	       cloud->points[i].x,
      // 	       cloud->points[i].y,
      // 	       cloud->points[i].z);
      filtered->points.push_back(cloud->points[i]);
    }
    
  }
}

//Filters out ground using a passthrough filter
void redblade_stereo::filterRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
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
  double n = (double)in->points.size();
  for(size_t i = 0; i<in->points.size();++i){
    x.push_back(in->points[i].x);
    y.push_back(in->points[i].y);
  }
  std::sort(x.begin(),x.end());
  std::sort(y.begin(),y.end());
  point.x = ((double)std::accumulate(x.begin(),x.end(),0.0))/n;
  point.y = ((double)std::accumulate(y.begin(),y.end(),0.0))/n;
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
  ROS_INFO("Size of input cloud %d",in->points.size());
  if(in->points.size()<sigSize){//Significance test
    return false;
  }
  Eigen::VectorXf coeff;
  coeff.resize(6);
  ransac(in,pole,coeff);
  ROS_INFO("Size of coefficients %d",coeff.size());
  if(coeff.size()==0){
    return false;
  }  
  ROS_INFO("Model Coefficients (x:%f,y:%f,z:%f)+(dx:%f,dy:%f,dz:%f)",
   	   coeff[0],coeff[1],coeff[2],coeff[3],coeff[4],coeff[5]);
  if(fabs(coeff[5])>verticalTolerance){     //Vertical line test
    if(pole->points.size()>sigSize){//Significance test
      // int clusters = cluster(pole,0.01); //1 cm
      // if(clusters==1){              //Test to see if the line is one contiguous segment
      ROS_INFO("Size of line %d",pole->points.size());
      // ROS_INFO("Model Coefficients (x:%f,y:%f,z:%f)+(dx:%f,dy:%f,dz:%f)",
      // 	       coeff[0],coeff[1],coeff[2],coeff[3],coeff[4],coeff[5]);
      return true;}}
  //}
  return false;
}
