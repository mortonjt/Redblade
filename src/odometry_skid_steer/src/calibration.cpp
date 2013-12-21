#include <string>
#include <cmath>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

ros::Time prev_time;
nav_msgs::Odometry odom;
std::string odom_frame_id;
bool imu_init = false;
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

  double dTheta = orientation.z - prev_orientation.z;
  theta+=dTheta;
  double dTime = (front_msg.encoders.time_delta+back_msg.encoders.time_delta)/2.0;
  intVl+=dTime*
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
  //See odometry_skid_steer.h for all constants
  n.param("front_encoders", front_encoder_namespace, std::string("/front_encoders"));
  n.param("back_encoders", back_encoder_namespace, std::string("/back_encoders"));
  n.param("imu", imu_namespace, std::string("/imu"));
  n.param("wheel_base_width", wheel_base_width, 0.473);
  n.param("cmd_vel", cmd_namespace, "/cmd_vel");

  double theta = 0; //rotation angle
  double intVr = 0, intVl = 0; //integral of Vr and Vl

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
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  
  //Set up rate for cmd_vel topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz

  //publish cmd_vel topic every 25 ms (40 hz)
  while(ros::ok()){
      if(front_recv and back_recv and imu_init){
  	publish_loop();
	front_recv = false;
	back_recv = false;
      }
    //sleep for a bit to stay at 40 hz
    cmd_vel_rate.sleep();
  }

  spinner.stop();
  return(0);
}
