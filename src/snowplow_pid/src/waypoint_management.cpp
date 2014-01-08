#include "ros/ros.h"
#include "snowplow_pid/request_next_waypoints.h"
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

//Constants for indexing waypoints
#define X 0
#define Y 1
#define FWD 2

struct Waypoint{
  double x,y;
  bool forward;
};

std::string waypoints_filename;
std::vector<Waypoint> waypoints;
Waypoint start, dest;
int waypoint_number;

void split_to_double(const std::string &s, char delim, std::vector<double> &elements){
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss,item,delim)){
    elements.push_back(atof(item.c_str()));
  }
}

bool read_in_waypoints(){
  //first, read in each line
  std::vector<std::string> elements;
  std::string item;
  std::ifstream file(waypoints_filename.c_str());
  while(std::getline(file, item, '\n')){
    elements.push_back(item);
    ROS_INFO("Read item:%s", (char*)item.c_str());
  }

  //split each line
  for(int i = 0; i < elements.size(); i++){
    std::vector<double> delimited_line;
    split_to_double(elements[i], ',', delimited_line);
    Waypoint temp;
    temp.x = delimited_line[0];
    temp.y = delimited_line[1];
    temp.forward = delimited_line[2];
    waypoints.push_back(temp);
  }
  
  return true;
}

bool next_waypoint(){
  //check to make sure we have a valid waypoint to give, if not, loop back to beginning
  //TODO

  if(waypoints.size() > 0){
    //populate waypoints
    start.x = waypoints[0].x;
    start.y = waypoints[0].y;
    dest.x = waypoints[1].x;
    dest.y = waypoints[1].y;
    forward = waypoints[1].forward;

    // remove waypoint that we've reached. should probably remove this
    // from inside the reached destination method.
    waypoints.pop_front();

    ROS_INFO("Next waypoints request received");
  }
  else{
    ROS_WARN("No more waypoints!");
  }
  //waypoint_number++;
  
return true;
}

void split_to_double(const std::string &s, char delim, std::vector<double> &elements){
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss,item,delim)){
    elements.push_back(atof(item.c_str()));
  }
}

bool read_in_waypoints(){
  //first, read in each line
  std::vector<std::string> elements;
  std::string item;
  std::ifstream file(waypoints_filename.c_str());
  while(std::getline(file, item, '\n')){
    elements.push_back(item);
    ROS_INFO("Read item:%s", (char*)item.c_str());
  }

  //split each line
  for(int i = 0; i < elements.size(); i++){
    std::vector<double> delimited_line;
    split_to_double(elements[i], ',', delimited_line);
    Waypoints temp;
    temp.x = delimited_line[0];
    temp.y = delimited_line[1];
    temp.forward = delimited_line[2];
    waypoints.push_back(temp);
  }
  
  return true;
}

bool next_waypoint(){
  //check to make sure we have a valid waypoint to give, if not, loop back to beginning
  //TODO

  if(waypoints.size() > 0){
    //populate waypoints
    start.x = waypoints[0].x;
    start.y = waypoints[0].y;
    dest.x = waypoints[1].x;
    dest.y = waypoints[1].y;
    forward = waypoints[1].forward;

    // remove waypoint that we've reached. should probably remove this
    // from inside the reached destination method.
    waypoints.pop_front();

    ROS_INFO("Next waypoints request received");
  }
  else{
    ROS_WARN("No more waypoints!");
  }
  
return true;
}



