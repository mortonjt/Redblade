#include "redblade_laser.h"

/*
  Laser Orientation                            Robot Orientation	       	   
    y                                           	        x		
    ^					        	 	^	
    |                                                           |                   
    |                                                           |                    
    |				     	        		|	
    |				     	        		|	
    |				     	        	        | 		
    *----------------> x	                y---------------* 

*/

bool redblade_laser::inBounds(double x, double y){
  double minx = *std::min_element(this->x.begin(),this->x.end());
  double maxx = *std::max_element(this->x.begin(),this->x.end());
  double miny = *std::min_element(this->y.begin(),this->y.end());
  double maxy = *std::max_element(this->y.begin(),this->y.end());

  return (x>minx and x<maxx and y>miny and y<maxy);
}

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
}

void redblade_laser::addScan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  queue.push_back(cloud);
  if(queue.size()>this->maxSize){
    queue.pop_front();
  }
}
