#include "ros/ros.h"
#include "snowplow_pid/request_next_waypoints.h"

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

  ros::ServiceServer = n.advertiseService("request_next_waypoints", next_waypoints);
  ROS_INFO("Ready to provide next waypoints.");
  ros::spin();
 
  return 0;
}
