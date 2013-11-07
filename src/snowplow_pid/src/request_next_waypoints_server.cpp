#include "ros/ros.h"
#include "snowplow_pid/request_next_waypoints.h"
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

std::string waypoints_filename;
std::vector<std::vector<double> > waypoints;

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
    //S_INFO("1: %s", item.c_str());
    elements.push_back(item);
  }

  //split each line
  for(int i = 0; i < elements.size(); i++){
    std::vector<double> delimited_line;
    split_to_double(elements[i], ',', delimited_line);
    waypoints.push_back(delimited_line);
  }
  
  return true;
}

bool next_waypoints(snowplow_pid::request_next_waypoints::Request &req,
		    snowplow_pid::request_next_waypoints::Response &res){
  res.start.x = 10;
  res.start.y = 5;
  res.dest.x = 0;
  res.dest.y = -5;
  res.forward = 1;
  ROS_INFO("Next waypoints request received");
  return true;
}

int main(int argc, char** argv){
  ros::init (argc, argv, "request_next_waypoints_server");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  //TODO: add waypoints vector and populate this vector with waypoints
  //read in from some file that was passed in as a parameter
  nh.param("waypoints_filename", waypoints_filename, std::string("waypoints.txt"));
  ROS_INFO("Waypoints file:%s", waypoints_filename.c_str());
  bool file_good = read_in_waypoints();

  if(!file_good){
    ROS_ERROR("Error in waypoints service.");
    return 1;
  }
  
  for(int i = 0; i < waypoints.size(); i++){
    ROS_INFO("(%f, %f)\t%f", waypoints[i][0], waypoints[i][1], waypoints[i][2]);
  }

  ros::ServiceServer service = n.advertiseService("request_next_waypoints", next_waypoints);
  ROS_INFO("Ready to provide next waypoints.");
  ros::spin();
 
  return 0;
}
