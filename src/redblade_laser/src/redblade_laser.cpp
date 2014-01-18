#include "redblade_laser.h"
#include <iostream>

#define POLE_HITS 5
#define BAD_HITS 1
#define RANGE_THRESH 0.05
#define LASER_OFFSET 0.3


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
   >>x[1]>>y[1];
  h.close();
  fieldAngle = -atan2(y[1],x[1]);
}

redblade_laser::redblade_laser(std::string surveyFile,
			       bool searchSnowField,
			       bool tripleI,
			       double laserOffset,
			       int queueSize){
  this->surveyFile = surveyFile;
  this->laserOffset = laserOffset;  
  this->searchSnowField = searchSnowField;
  this->maxSize = queueSize;
  this->x.resize(4);
  this->y.resize(4);
  this->tripleI = tripleI;
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


void redblade_laser::scan2cloud(sensor_msgs::LaserScan scan_in,
				pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud){
  // laser_geometry::LaserProjection projector_;
  // projector_.projectLaser(currentScan, cloud); 
  // pcl::fromROSMsg(cloud,*pcl_cloud);

  int num_scans = (int)((scan_in.angle_max-scan_in.angle_min)/scan_in.angle_increment);
  //int num_scans = scan_in.ranges.size();
  for(int i = 0; i < num_scans; i++){
    if(scan_in.ranges[i]==INFINITY ||
       std::isnan(scan_in.ranges[i])){
      continue;
    }
    double r = scan_in.ranges[i];
    double theta = scan_in.angle_min + (scan_in.angle_increment*(i));
    pcl::PointXYZ pt;
    pt.x = r*cos(theta);
    pt.y = r*sin(theta);
    pt.z = 0;
    pcl_cloud->points.push_back(pt);
  }

  // //loop through each scan
  // int num_scans = (int)((scan_in.angle_max-scan_in.angle_min)/scan_in.angle_increment);
  // for(int i = 0; i < num_scans; i++){
  //   if(scan_in.ranges[i]==INFINITY ||
  //      std::isnan(scan_in.ranges[i])){
  //     continue;
  //   }
  //   //ROS_INFO("looping through scans %d", i);
    
  //   double current_range = scan_in.ranges[i];
  //   int good = 1;
  //   int bad = 0;
  //   int j = i+1;
  //   double r = current_range;
  //   while(1){
  //     //check for end of array
  //     //ROS_INFO("j: %d", j);
  //     //ROS_INFO("curr_scan: %f\n",scan_in.ranges[j]);
  //     if(j>=(num_scans-1)){
  // 	if(good >= POLE_HITS){
  // 	  //ROS_INFO("pole hits: %d", good);
  // 	  pcl::PointXYZ pt;
  // 	  //r = r/((double)(good));      
  // 	  r = current_range;
  // 	  double theta = scan_in.angle_min + (scan_in.angle_increment*((j+i)/2));
  // 	  pt.z = 0;
  // 	  pt.x = r*cos(theta);
  // 	  pt.y = r*sin(theta);
  // 	  pt.z = 0;
  // 	  pcl_cloud->points.push_back(pt);
  // 	  //ROS_INFO("Object Position: range->%f\tangle->%f", temp_point[0], temp_point[1]);
  // 	}
  // 	break;
  //     }else if(fabs(scan_in.ranges[j]-current_range) > RANGE_THRESH){
  // 	bad++;
  // 	if(bad <= BAD_HITS){
  // 	  continue;
  // 	}else{
  // 	  if(good >= POLE_HITS){
  // 	    //r = r/((double)(good));      
  // 	    pcl::PointXYZ pt;
  // 	    //std::accumulate(scan_in.ranges+i,scan_in.ranges+j,r);
  // 	    r = current_range;
	    
  // 	    double theta = scan_in.angle_min + (scan_in.angle_increment*((j+i)/2));
  // 	    pt.x = r*cos(theta);
  // 	    pt.y = r*sin(theta);
  // 	    pt.z = 0;
  // 	    pcl_cloud->points.push_back(pt);
  // 	    //ROS_INFO("pole hits: %d", good);
  // 	    //ROS_INFO("Object Position: range->%f\tangle->%f", temp_point[0], temp_point[1]);
  // 	    i = j;
  // 	  }
  // 	  break;
  // 	}
  //     }else if(fabs(scan_in.ranges[j]-current_range) < RANGE_THRESH){
  // 	r+=scan_in.ranges[j];
  // 	good++;
  //     }
  //     j++;
  //   }
  // }
  
  /*Copied from laser_geometry.cpp from laser_geometry.h on hydro*/
  // size_t n_pts = scan_in.ranges.size ();
  // Eigen::ArrayXXd ranges (n_pts, 2);
  // Eigen::ArrayXXd output (n_pts, 2);
  
  // // Get the ranges into Eigen format
  // for (size_t i = 0; i < n_pts; ++i){
  //   ranges (i, 0) = (double) scan_in.ranges[i];
  //   ranges (i, 1) = (double) scan_in.ranges[i];
  // }

  // // Check if our existing co_sine_map is valid
  // if (co_sine_map_.rows () != (int)n_pts || angle_min_ != scan_in.angle_min || angle_max_ != scan_in.angle_max )
  //   {
  //     ROS_DEBUG ("[projectLaser] No precomputed map given. Computing one.");
  //     co_sine_map_ = Eigen::ArrayXXd (n_pts, 2);
  //     angle_min_ = scan_in.angle_min;
  //     angle_max_ = scan_in.angle_max;
  //     // Spherical->Cartesian projection
  //     for (size_t i = 0; i < n_pts; ++i)
  // 	{
  // 	  co_sine_map_ (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
  // 	  co_sine_map_ (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
  // 	}
  //   }
  // output = ranges * co_sine_map_;
  // for(size_t i = 0; i<n_pts;++i){
  //   pcl_cloud->points[i].x = output(i,0);
  //   pcl_cloud->points[i].y = output(i,1);
  //   pcl_cloud->points[i].z = 0;
  // }
}


bool redblade_laser::saturated(){
  return this->maxSize==this->queue.size();
}

bool redblade_laser::inSnowField(double transformedX, double transformedY){
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

void redblade_laser::rotate(double& x, double& y){
  double transformedX = x*cos(this->fieldAngle)-y*sin(this->fieldAngle);
  double transformedY = x*sin(this->fieldAngle)+y*cos(this->fieldAngle);
  x = transformedX;
  y = transformedY;
}
bool redblade_laser::inBounds(double x, double y){
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
    
    if(sqrt((xc*xc)+(yc*yc))>1.0){//Filter out the supporting poles on the side of the robot
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
  point.x = x[n/2];
  point.y = y[n/2];
  point.z = 0;

  //point.x = totalx/((double)in->points.size());
  //point.y = totaly/((double)in->points.size());
  //point.z = totalz/((double)in->points.size());
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
  int curSize = 0;
  for(std::vector<pcl::PointIndices>::iterator it = cluster_indices.begin();
      it!=cluster_indices.end(); ++it){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); 
	pit!=it->indices.end();++pit){
      cloud_cluster->points.push_back(cloud->points[*pit]);
    }
    cloud_cluster->width = cluster->points.size();
    cloud_cluster->height = 1;
    if(cloud_cluster->points.size()>curSize){
      cluster->points.resize(cloud_cluster->points.size());
      std::copy(cloud_cluster->points.begin(),
		cloud_cluster->points.end(),
		cluster->points.begin());
      cloud_cluster->width = cluster->points.size();
      cloud_cluster->height = 1;
      curSize = cloud_cluster->points.size();
    }
  }
}

bool redblade_laser::findPole(geometry_msgs::Point& point,
			      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster,
			      double tolerance){
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    combined(new pcl::PointCloud<pcl::PointXYZ>());
  // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
  //   cluster(new pcl::PointCloud<pcl::PointXYZ>());
  this->getClouds(combined);
  //ROS_INFO("Size of combined %d",combined->points.size());
  if(combined->points.size()==0){
    return false;
  }
  this->cluster(combined,cluster,tolerance);
  //ROS_INFO("Size of cluster %d",cluster->points.size());
  if(cluster->points.size() < this->maxSize){
    return false;
  }
  this->cloud2point(cluster,point);
  //this->cloud2point(combined,point);
  if(isnan(point.x) or isnan(point.y) or isnan(point.x)){
    return false;
  }
  return true;
}
