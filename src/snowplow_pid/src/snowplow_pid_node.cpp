#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>

//global stuff
ros::Publisher cmd_vel_pub;

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "snowplow_pid_node");
  ros::NodeHandle n;

  //Subscribe to GPS topic
  //TODO

  //Subscribe to IMU topic
  //TODO

  //Set up cmd_vel publisher
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  //Set up rate for cmd_vel_pub topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz

  while(ros::ok()){
    
    cmd_vel_rate.sleep();
  }

}
