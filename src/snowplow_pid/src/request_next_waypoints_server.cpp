#include "ros/ros.h"
#include "snowplow_pid/request_next_waypoints.h"

std::string waypoints_filename;

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

  ros::ServiceServer service = n.advertiseService("request_next_waypoints", next_waypoints);
  ROS_INFO("Ready to provide next waypoints.");
  ros::spin();
 
  return 0;
}
