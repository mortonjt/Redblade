#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <cstdio>
#include <string>
#include <fstream>
#include <thread>
#include <math.h>
#include <vector>
#include <deque>
#include "survey_snow_field.h"

//global variable pointbuffer. callback keeps this filled with most recent points
std::deque<Coordinates> pointBuffer;
int numDataPoints, numCorners;

void writeData(std::string filename1, std::vector<Coordinates>& cornerList){

  std::ofstream fs1(filename1.c_str(),std::ofstream::out|std::ofstream::app);
 
  if(!fs1.good()){
    std::cout << "ERROR OPENING FILE: " << filename1 << std::endl;
  }

  //set float precision to exactly 10
  fs1.unsetf(std::ofstream::floatfield);
  fs1.precision(10);
  fs1.setf(std::ofstream::fixed,std::ofstream::floatfield);

  for(int i = 0; i < numCorners; i++){
    fs1 << cornerList[i].lat << ", " << cornerList[i].lon 
	<< std::endl;
  }

  fs1.close();
}

Coordinates mean(std::deque<Coordinates> coords){
  double sumlat = 0;
  double sumlon = 0;
  double sumh = 0;
  int numCoords = coords.size();
  for(int i = 0; i < numCoords; i++){
    sumlat += coords[i].lat;
    sumlon += coords[i].lon;
    sumh += coords[i].h;
  }
  return Coordinates(sumlat/numCoords,sumlon/numCoords,sumh/numCoords);
}

bool isPrecise(std::deque<Coordinates> coords, Coordinates avg){
  double devlat = 0;
  double devlon = 0;
  double numCoords = coords.size();

  for(int i = 0; i < numCoords; i++){
    devlat += (coords[i].lat - avg.lat)
      * (coords[i].lat - avg.lat);
    devlon += (coords[i].lon - avg.lon) 
      * (coords[i].lon - avg.lon);
  }

  devlat = 1/numCoords * devlat;
  devlon = 1/numCoords * devlon;

  if(devlat > gpsTolerance || devlon > gpsTolerance){
    std::cout << "STD of points not within tolerance. hold it steady." << std::endl;
    return false;
  }

  return true;
}

void collectCornerPoints(std::vector<Coordinates>& cornerList){

  std::string confirm;
  //  Coordinates referenceCoord;
  std::deque<Coordinates> selectPoints;
  Coordinates avg;
  int ii = 0;

  while(cornerList.size() < numCorners){
    if(pointBuffer.size() > numDataPoints){
      
      std::cout << "Press Enter Key to take point #" << ii << std::endl;
      getline(std::cin, confirm);
      
      do{//makes sure points are under a certain std dev
	selectPoints = std::deque<Coordinates>(pointBuffer);
	avg = mean(selectPoints);
      }while(!isPrecise(selectPoints,avg));
      
      cornerList.push_back(Coordinates(avg.lat, avg.lon, avg.h));

      // //if this is the first point, make it the reference for all others
      // if(cornerList.size() == 0)
      // 	referenceCoord = Coordinates(avg.lat,avg.lon,avg.h);

      // cornerList.back().setReference(referenceCoord);
      pointBuffer.clear();
      ii++;
    }//end if

  }//end while
}

void queueGPSCallback(const sensor_msgs::NavSatFix::ConstPtr& point){
  
  //just make sure we're pulling gps data from the right RX
  if(point->header.frame_id == "/HiperLite_rover"){
    pointBuffer.push_front(Coordinates(point->latitude,point->longitude,point->altitude));
    if(pointBuffer.size() > numDataPoints)
      pointBuffer.pop_back();
  }

}

int main(int argc, char** argv){
  //Node setup

  ros::init(argc, argv, "survey_snow_field");
  ros::NodeHandle n;//global namespace
  ros::NodeHandle nh("~");//local namespace, used for params

  std::string filepath,skip;
  nh.param("filepath", filepath, skip);
  nh.param("numDataPoints",numDataPoints,-1);
  nh.param("numCorners",numCorners,-1);

  //create filename strings
  std::string filename1(filepath+"survey_geodetic.csv");

  //delete files if the already exist
  std::remove(filename1.c_str());

  //create coordinate buffer and corner list
  std::vector<Coordinates> cornerList;

  std::thread cornerCollector(collectCornerPoints, std::ref(cornerList));

  //Subscribe to GPS topic
  ros::Subscriber fix_sub = n.subscribe("/fix", 10, queueGPSCallback);

  while(cornerList.size() < numCorners){
    ros::spinOnce();
  }

  cornerCollector.join();

  writeData(filename1, cornerList);

  return 0;
}
