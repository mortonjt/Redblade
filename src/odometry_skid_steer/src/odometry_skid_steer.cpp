#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <cmath>

static double clicks_per_m = 15768.6;
static double wheel_base_width = 0.473;
static double wheel_base_length;
ros::Publisher robo_front_pub;
ros::Publisher robo_back_pub;
nav_msgs::Odometry odom;
ros::Publisher odom_pub;

void publish_loop(){
  //publish odom messages
  odom_pub.publish(odom);
}  
void frontEncoderCallback(const geometry_msgs::Twist::ConstPtr& msg){

}
void backEncoderCallback(const geometry_msgs::Twist::ConstPtr& msg){

}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_skid_steer");
  ros::NodeHandle n; //in the global namespace

  //Start Spinner so that encoder Callbacks happen in a seperate thread
  ros::AsyncSpinner spinner(2);

  //Subscribe to front/back encoder topics
  ros::Subscriber front_encoder_sub = n.subscribe("cmd_vel", 1, frontEncoderCallback);
  ros::Subscriber back_encoder_sub = n.subscribe("cmd_vel", 1, backEncoderCallback);
  odom_pub = n.advertise<geometry_msgs::Twist>("roboteq_front/cmd_vel", 10);

  //Set up rate for cmd_vel topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz

  //publish cmd_vel topic every 25 ms (40 hz)
  while(ros::ok()){
    publish_loop();

    //sleep for a bit to stay at 40 hz
    cmd_vel_rate.sleep();
  }

  spinner.stop();

};
