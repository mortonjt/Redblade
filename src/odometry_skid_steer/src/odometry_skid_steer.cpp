//#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <cmath>
#include "redblade_ax2550/StampedEncoders.h"
#include <tf/tf.h>

double x_pos = 0, y_pos = 0, theta = 0;
double x_vel = 0, y_vel = 0, theta_vel = 0;

ros::Time prev_time;
double rot_cov = 0.0;
double pos_cov = 0.0;

static double wheel_circumference = 0.0;
static double wheel_diameter = 0.0;

static double clicks_per_m = 15768.6;
static double wheel_base_width = 0.473;
static double wheel_base_length;
nav_msgs::Odometry odom;

ros::Publisher odom_pub;

std::string odom_frame_id;

redblade_ax2550::StampedEncoders front_encoders,back_encoders;

bool front_recv = false;
bool back_recv = false;

void frontEncoderCallback(const redblade_ax2550::StampedEncoders& msg){
  front_encoders = msg;
  front_recv = true;
}

void backEncoderCallback(const redblade_ax2550::StampedEncoders& msg){
  back_encoders = msg;
  back_recv = true;
}

void getDeltaAnglePos(redblade_ax2550::StampedEncoders front_msg,
 		      redblade_ax2550::StampedEncoders back_msg,
 		      double& delta_time,
 		      double& distance_delta,
 		      double& theta_delta){
//   double delta_time1 = front_encoders.encoders.time_delta;
//   double delta_front_left_encoders = front_encoders.encoders.left_wheel;
//   double delta_front_right_encoders = front_encoders.encoders.right_wheel;
  
//   double delta_time2 = back_encoders.encoders.time_delta;
//   double delta_back_left_encoders = back_encoders.encoders.left_wheel;
//   double delta_back_right_encoders = back_encoders.encoders.right_wheel;

//   //Combine both wheels into "bigger" wheels
//   double left_encoders  = (delta_front_left_encoders  + delta_back_left_encoders)/2;
//   double right_encoders = (delta_front_right_encoders + delta_back_right_encoders)/2;

//   delta_time = (delta_time1+delta_time2)/2;  // The average delta time
//   distance_delta = (left_encoders+right_encoders)/(clicks_per_m);
//   theta_delta = (left_encoders-right_encoders)/wheel_base_width;
}

void publishOdometry(){
//  //next, we'll publish the odometry message over ROS
//   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
//   nav_msgs::Odometry odom;
//   odom.header.stamp = ros::Time::now();
//   odom.header.frame_id = odom_frame_id;
 
//   //set the position
//   odom.pose.pose.position.x = x_pos;
//   odom.pose.pose.position.y = y_pos;
//   odom.pose.pose.position.z = 0.0;
//   odom.pose.pose.orientation = odom_quat;
 
//   //set the velocity
//   odom.child_frame_id = "base_link";
//   odom.twist.twist.linear.x = x_vel;
//   odom.twist.twist.linear.y = y_vel;
//   odom.twist.twist.angular.z = theta_vel;
 
//   //TODO: covariance stuff
//   odom.pose.covariance[0] = pos_cov;
//   odom.pose.covariance[7] = pos_cov;
//   odom.pose.covariance[14] = 1e100;
//   odom.pose.covariance[21] = 1e100;
//   odom.pose.covariance[28] = 1e100;
//   odom.pose.covariance[35] = rot_cov;

//   //publish the message
//   odom_pub.publish(odom);
 
}

void publish_loop(){
//   double delta_time;
//   double distance_delta;
//   double theta_delta;   
//   //publish odom messages
//   getDeltaAnglePos(front_encoders,
// 		   back_encoders,
// 		   delta_time,	
// 		   distance_delta,
// 		   theta_delta);
  
//   //Is this orientation right???  Should the sin/cos be flipped?
//   double dX = distance_delta*cos(theta_delta);
//   double dY = distance_delta*sin(theta_delta);  
  
//   x_pos+=dX;
//   y_pos+=dY;
//   theta+=theta_delta;

//   x_vel = dX/delta_time;
//   y_vel = dY/delta_time;
//   theta_vel = theta_delta/delta_time;

//   publishOdometry();
}  


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_skid_steer");
  ros::NodeHandle n; //in the global namespace
  ros::NodeHandle nh("~");//local namespace, used for params
  std::string front_encoder_namespace,back_encoder_namespace;
  n.param("front_encoders", front_encoder_namespace, std::string("/front_encoders"));
  n.param("back_encoders", back_encoder_namespace, std::string("/back_encoders"));
  n.param("rotation_covariance",rot_cov, 1.0);
  n.param("position_covariance",pos_cov, 1.0);
  n.param("odom_frame_id", odom_frame_id, std::string("odom"));
  n.param("wheel_base_length", wheel_base_length, 0.473);
  
  //Start Spinner so that encoder Callbacks happen in a seperate thread
  ros::AsyncSpinner spinner(2);
  spinner.start();

  //Subscribe to front/back encoder topics
  ros::Subscriber front_encoder_sub = n.subscribe(front_encoder_namespace, 1, 
  						  frontEncoderCallback);
  ros::Subscriber back_encoder_sub = n.subscribe(back_encoder_namespace, 1, 
  						 backEncoderCallback);
  odom_pub = n.advertise<geometry_msgs::Twist>("roboteq_front/cmd_vel", 10);
  
  //Set up rate for cmd_vel topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz

  //publish cmd_vel topic every 25 ms (40 hz)
  while(ros::ok()){
      if(front_recv and back_recv){
  	publish_loop();
      }
    //sleep for a bit to stay at 40 hz
    cmd_vel_rate.sleep();
  }

  spinner.stop();
  return(0);
}
