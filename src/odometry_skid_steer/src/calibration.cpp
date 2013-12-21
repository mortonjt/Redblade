#include <string>
#include <cmath>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "odometry_skid_steer.h"

ros::Time prev_time;
nav_msgs::Odometry odom;
std::string odom_frame_id;
bool imu_init = false;
bool front_recv = false, back_recv=false;
geometry_msgs::Vector3 orientation;
geometry_msgs::Vector3 prev_orientation;
geometry_msgs::Twist cmd_velocity;
redblade_ax2550::StampedEncoders front_encoders,back_encoders;

ros::Publisher cmd_vel_pub;


void frontEncoderCallback(const redblade_ax2550::StampedEncoders& msg){
  front_encoders = msg;
  front_recv = true;
}

void backEncoderCallback(const redblade_ax2550::StampedEncoders& msg){
  back_encoders = msg;
  back_recv = true;
}

void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg){
  orientation.x = msg->x;
  orientation.y = msg->y;
  orientation.z = msg->z;
  imu_init = true;    

}

void publish_loop(odometry_skid_steer odomSS,double &theta, double &intVr, double &intVl){  
  double delta_time,left_encoders,right_encoders;
  double dTheta = orientation.z - prev_orientation.z;
  odomSS.getEncoders(front_encoders,back_encoders,left_encoders,right_encoders,delta_time);
  double Vr = (right_encoders/clicks_per_m)/delta_time;
  double Vl = (left_encoders/clicks_per_m)/delta_time;
  theta+=dTheta;
  intVl+=delta_time*Vl;
  intVr+=delta_time*Vr;
  double eff_wheel_base_width = (intVr-intVl)/theta;
  ROS_INFO("Effective Wheel Base Width %f",eff_wheel_base_width);
  cmd_velocity.linear.x = 0;
  cmd_velocity.linear.y = 0;
  cmd_velocity.linear.z = 0;
  cmd_velocity.angular.x = 0;
  cmd_velocity.angular.y = 0;
  cmd_velocity.angular.z = 1;
  cmd_vel_pub.publish(cmd_velocity);
  prev_orientation = orientation;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_skid_steer");
  ros::NodeHandle n; //in the global namespace
  ros::NodeHandle nh("~");//local namespace, used for params
  std::string front_encoder_namespace,back_encoder_namespace,imu_namespace,cmd_vel_namespace;
  double rot_cov_,pos_cov_;
  double wheel_base_width;
  std::string cmd_namespace;
  //See odometry_skid_steer.h for all constants
  n.param("front_encoders", front_encoder_namespace, std::string("/front_encoders"));
  n.param("back_encoders", back_encoder_namespace, std::string("/back_encoders"));
  n.param("imu", imu_namespace, std::string("/imu"));
  n.param("wheel_base_width", wheel_base_width, 0.473);
  n.param("cmd_vel", cmd_namespace, std::string("/cmd_vel"));

  double theta = 0; //rotation angle
  double intVr = 0, intVl = 0; //integral of Vr and Vl
  odometry_skid_steer odomSS(odom_frame_id,rot_cov_,pos_cov_,wheel_base_width);
    
  //Start Spinner so that encoder Callbacks happen in a seperate thread
  ros::AsyncSpinner spinner(3);
  spinner.start();

  //Subscribe to front/back encoder topics
  ros::Subscriber front_encoder_sub = n.subscribe(front_encoder_namespace, 1, 
  						  frontEncoderCallback);
  ros::Subscriber back_encoder_sub = n.subscribe(back_encoder_namespace, 1, 
  						 backEncoderCallback);
  ros::Subscriber imu_sub = n.subscribe(imu_namespace, 1, 
					imuCallback);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_namespace, 10);
  
  //Set up rate for cmd_vel topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz

  //publish cmd_vel topic every 25 ms (40 hz)
  while(ros::ok()){
      if(front_recv and back_recv and imu_init){
	publish_loop(odomSS,theta,intVr,intVl);
  	front_recv = false;
	back_recv = false;
      }
    //sleep for a bit to stay at 40 hz
    cmd_vel_rate.sleep();
  }

  spinner.stop();
  return(0);
}
