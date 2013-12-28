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
  front_encoders.encoders.right_wheel *= -1;
  front_encoders.encoders.left_wheel  *= -1;
  front_recv = true;
  //std::cout<<"Front right: "<<msg.encoders.right_wheel<<" Front left: "<<msg.encoders.left_wheel<<std::endl;
  //ROS_INFO("Front encoders received");
}

void backEncoderCallback(const redblade_ax2550::StampedEncoders& msg){
  back_encoders = msg;
  back_recv = true;
  //std::cout<<"Back right: "<<msg.encoders.right_wheel<<" Back left: "<<msg.encoders.left_wheel<<std::endl;
  //ROS_INFO("Back encoders received");
}

void imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  orientation.x = msg->vector.x;
  orientation.y = msg->vector.y;
  orientation.z = msg->vector.z;
  imu_init = true;    
  //ROS_INFO("Imu message received");
}

void estimate(odometry_skid_steer& odomSS,double &theta, double &intVr, double &intVl){  
  double delta_time,left_encoders,right_encoders;
  double dTheta = orientation.z - prev_orientation.z;
  odomSS.getEncoders(front_encoders,back_encoders,left_encoders,right_encoders,delta_time);
  intVr = right_encoders/clicks_per_m;
  intVl = left_encoders/clicks_per_m;
  theta+=dTheta;
  double eff_wheel_base_width = (intVr-intVl)/theta;
  ROS_INFO("dTheta %f theta %f",dTheta,theta);
  ROS_INFO("Effective Wheel Base Width %f",eff_wheel_base_width);
  ROS_INFO("Front Right %ld Back Right %ld Front Left %ld Back Left %ld",
  	   odomSS.prev_fr_encoder,
  	   odomSS.prev_br_encoder,
   	   odomSS.prev_fl_encoder,
  	   odomSS.prev_bl_encoder);
  
  prev_orientation = orientation;
  odomSS.update(front_encoders,
		back_encoders,
		orientation);
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
  nh.param("front_encoders", front_encoder_namespace, std::string("/front_encoders"));
  nh.param("back_encoders", back_encoder_namespace, std::string("/back_encoders"));
  nh.param("imu", imu_namespace, std::string("/imu"));
  nh.param("wheel_base_width", wheel_base_width, 0.473);

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

  ROS_INFO("front_encoder_namespace %s",front_encoder_namespace.c_str());
  ROS_INFO("back_encoder_namespace %s",back_encoder_namespace.c_str());
  ROS_INFO("imu_namespace %s",imu_namespace.c_str());
  ROS_INFO("Started calibration");
  //publish cmd_vel topic every 25 ms (40 hz)
  while(ros::ok()){
    if(front_recv and back_recv and imu_init){
      estimate(odomSS,theta,intVr,intVl);
      front_recv = false;
      back_recv = false;
    }
  }

  spinner.stop();
  return(0);
}
