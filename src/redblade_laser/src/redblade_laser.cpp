#include "redblade_laser.h"
#include <iostream>


redblade_laser::redblade_laser(std::string surveyFile,
			       double laserOffset,
			       int queueSize){
  this->surveyFile = surveyFile;
  this->laserOffset = laserOffset;  
  this->maxSize = queueSize;
  this->x.resize(4);
  this->y.resize(4);
  std::ifstream h((char*)surveyFile.c_str());
  h>>x[0]>>y[0]
   >>x[1]>>y[1]
   >>x[2]>>y[2]
   >>x[3]>>y[3];
  h.close();
}


void redblade_laser::scan2cloud(sensor_msgs::LaserScan::Ptr,
				pcl::PointCloud<pcl::PointXYZ>::Ptr){
  
}


bool redblade_laser::saturated(){
  return this->maxSize==this->queue.size();
}

bool redblade_laser::inBounds(double x, double y){
  double minx = *std::min_element(this->x.begin(),this->x.end());
  double maxx = *std::max_element(this->x.begin(),this->x.end());
  double miny = *std::min_element(this->y.begin(),this->y.end());
  double maxy = *std::max_element(this->y.begin(),this->y.end());
  
  //ROS_INFO("minx %lf maxx %lf miny %lf maxy %lf - (x:%lf,y:%lf)",minx,maxx,miny,maxy,x,y);
  return (x>minx and x<maxx and y>miny and y<maxy);
}

/*Transforms laser coordinates into ENU coordinates*/
void redblade_laser::transformLaser2ENU(geometry_msgs::Pose2D& currentPose,
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  double x0      = currentPose.x;
  double y0      = currentPose.y;
  double theta   = currentPose.theta;
  for(size_t i = 0; i<cloud->points.size();++i){
    double xc = cloud->points[i].x+this->laserOffset;
    double yc = cloud->points[i].y;
    // ROS_INFO("LAZER x %f y %f z %f",
    // 	     cloud->points[i].x,
    // 	     cloud->points[i].y,
    // 	     cloud->points[i].z);    
    // ROS_INFO("Robot xc %f yc %f",
    // 	     xc,
    // 	     yc);

    if(xc>0.4){//Filter out the supporting poles on the side of the robot
      cloud->points[i].x = xc*cos(theta)-yc*sin(theta)+x0;
      cloud->points[i].y = xc*sin(theta)+yc*cos(theta)+y0;
      cloud->points[i].z = 0;   
      // ROS_INFO("ENU theta %f x %f y %f z %f",
      // 	       theta,
      // 	       cloud->points[i].x,
      // 	       cloud->points[i].y,
      // 	       cloud->points[i].z);
    }else{
      cloud->points[i].x = 0xffff;
      cloud->points[i].y = 0xffff;
      cloud->points[i].z = 0xffff;
    }
  }
}

/*Filters everything outside of survey field*/
void redblade_laser::filterBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
				      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered){
  for(size_t i = 0; i<cloud->points.size();++i){
    if(this->inBounds(cloud->points[i].x,
		      cloud->points[i].y)){
      filtered->points.push_back(cloud->points[i]);
    }    
  }
}

void redblade_laser::addScan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  queue.push_back(cloud);
  if(queue.size()>this->maxSize){
    queue.pop_front();
  }
}

void redblade_laser::getClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  cloud->points.resize(0);
  for( std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = this->queue.begin();
       it != this->queue.end(); ++it){
    (*cloud)+=**it;
  }       
}

void redblade_laser::cloud2point(pcl::PointCloud<pcl::PointXYZ>::Ptr in,
				 geometry_msgs::Point& point){
  double totalx=0,totaly=0,totalz=0;
  int n = in->points.size();
  for(size_t i = 0; i<in->points.size();++i){
    totalx+= in->points[i].x;
    totaly+= in->points[i].y;
    totalz+= in->points[i].z;
  }
  //Use the median
  // point.x = x[n/2];
  // point.y = y[n/2];
  // point.z = 0;

  point.x = totalx/((double)in->points.size());
  point.y = totaly/((double)in->points.size());
  point.z = totalz/((double)in->points.size());
}


void redblade_laser::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			     pcl::PointCloud<pcl::PointXYZ>::Ptr cluster,
			     double tolerance){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  int maxSize = 0;
  for(std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin();
      it!=cluster_indices.end(); ++it){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); 
	pit!=it->indices.end();++pit){
      cloud_cluster->points.push_back(cloud->points[*pit]);
    }
    cloud_cluster->width = cluster->points.size();
    cloud_cluster->height = 1;
    if(cloud_cluster->points.size()>maxSize){
      cluster->points.resize(cloud_cluster->points.size());
      std::copy(cloud_cluster->points.begin(),
		cloud_cluster->points.end(),
		cluster->points.begin());
      cloud_cluster->width = cluster->points.size();
      cloud_cluster->height = 1;
      maxSize = cloud_cluster->points.size();
    }
  }
}

bool redblade_laser::findPole(geometry_msgs::Point& point,double tolerance){
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    combined(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cluster(new pcl::PointCloud<pcl::PointXYZ>());
  this->getClouds(combined);
  if(combined->points.size()==0){
    return false;
  }
  // this->cluster(combined,cluster,tolerance);
  // this->cloud2point(cluster,point);
  this->cloud2point(combined,point);
  if(isnan(point.x) or isnan(point.y) or isnan(point.x)){
    return false;
  }
  return true;
}
