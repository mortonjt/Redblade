//#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <cmath>
//#include "ax2550/StampedEncoders.h"
#include <tf/tf.h>
#include "odometry_skid_steer.h"


ros::Time prev_time;
nav_msgs::Odometry odom;
ros::Publisher odom_pub;
std::string odom_frame_id;

ax2550::StampedEncoders front_encoders,back_encoders;

bool front_recv = false;
bool back_recv = false;

void frontEncoderCallback(const ax2550::StampedEncoders& msg){
  front_encoders = msg;
  front_recv = true;
}

void backEncoderCallback(const ax2550::StampedEncoders& msg){
  back_encoders = msg;
  back_recv = true;
}

odometry_skid_steer::odometry_skid_steer(double rot_cov_, double pos_cov_){
  x_pos = 0, y_pos = 0, theta = 0;
  x_vel = 0, y_vel = 0, theta_vel = 0;
  rot_cov = rot_cov_;
  pos_cov = pos_cov_;
}
odometry_skid_steer::~odometry_skid_steer(){

}

void odometry_skid_steer::getDeltaAnglePos(ax2550::StampedEncoders front_msg,
					   ax2550::StampedEncoders back_msg,
					   double& delta_time,
					   double& distance_delta,
					   double& theta_delta){
  double delta_time1 = front_encoders.encoders.time_delta;
  double delta_front_left_encoders = front_encoders.encoders.left_wheel;
  double delta_front_right_encoders = front_encoders.encoders.right_wheel;
  
  double delta_time2 = back_encoders.encoders.time_delta;
  double delta_back_left_encoders = back_encoders.encoders.left_wheel;
  double delta_back_right_encoders = back_encoders.encoders.right_wheel;

  //Combine both wheels into "bigger" wheels such wheels
  double left_encoders  = (delta_front_left_encoders  + delta_back_left_encoders)/2;
  double right_encoders = (delta_front_right_encoders + delta_back_right_encoders)/2;

  delta_time = (delta_time1+delta_time2)/2;  // The average delta time
  distance_delta = (left_encoders+right_encoders)/(clicks_per_m);
  theta_delta = (left_encoders-right_encoders)/wheel_base_width;
}

nav_msgs::Odometry odometry_skid_steer::getOdometry(){
   //next, we'll publish the odometry message over ROS
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;
  
    //set the position
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
  
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = x_vel;
    odom.twist.twist.linear.y = y_vel;
    odom.twist.twist.angular.z = theta_vel;
  
    //TODO: covariance stuff lul
    odom.pose.covariance[0] = pos_cov;
    odom.pose.covariance[7] = pos_cov;
    odom.pose.covariance[14] = 1e100;
    odom.pose.covariance[21] = 1e100;
    odom.pose.covariance[28] = 1e100;
    odom.pose.covariance[35] = rot_cov;
    return odom;
}


void odometry_skid_steer::update(double delta_time,
				 double distance_delta,
				 double theta_delta){
  //Is this orientation right???  Should the sin/cos be flipped?
  double dX = distance_delta*cos(theta_delta);
  double dY = distance_delta*sin(theta_delta);  
  
  x_pos+=dX;
  y_pos+=dY;
  theta+=theta_delta;

  x_vel = dX/delta_time;
  y_vel = dY/delta_time;
  theta_vel = theta_delta/delta_time;

}

void publish_loop(odometry_skid_steer odomSS){
  double delta_time;
  double distance_delta;
  double theta_delta;   
  //publish odom messages
  odomSS.getDeltaAnglePos(front_encoders,
			  back_encoders,
			  delta_time,	
			  distance_delta,
			  theta_delta);
  odomSS.update(delta_time,	
		distance_delta,
		theta_delta);
  
  nav_msgs::Odometry odom = odomSS.getOdometry();
  odom_pub.publish(odom);
  
}  


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_skid_steer");
  ros::NodeHandle n; //in the global namespace
  ros::NodeHandle nh("~");//local namespace, used for params
  std::string front_encoder_namespace,back_encoder_namespace;
  double rot_cov_,pos_cov_;
  //See odometry_skid_steer.h for all constants
  n.param("front_encoders", front_encoder_namespace, std::string("/front_encoders"));
  n.param("back_encoders", back_encoder_namespace, std::string("/back_encoders"));
  n.param("rotation_covariance",rot_cov_, 1.0);
  n.param("position_covariance",pos_cov_, 1.0);
  n.param("odom_frame_id", odom_frame_id, std::string("odom"));
  n.param("wheel_base_length", wheel_base_length, 0.473);
  odometry_skid_steer odomSS(rot_cov_,pos_cov_);

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
  	publish_loop(odomSS);
      }
    //sleep for a bit to stay at 40 hz
    cmd_vel_rate.sleep();
  }

  spinner.stop();
  return(0);
}
