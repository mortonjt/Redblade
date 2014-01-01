//#include <ros/ros.h>
#include <string>
#include <cmath>
//#include "ax2550/StampedEncoders.h"
#include "odometry_skid_steer.h"

//Constants
// static double wheel_circumference = 0.0;
// static double wheel_diameter = 0.0;
// static double clicks_per_m = 15768.6;
// static double wheel_base_width = 0.473;
// static double eff_wheel_base_width;
// static double wheel_base_length;

ros::Time prev_time;
nav_msgs::Odometry odom;
ros::Publisher odom_pub;
std::string odom_frame_id;
bool imu_init = false;
geometry_msgs::Vector3 orientation;
//geometry_msgs::Vector3 prev_orientation;

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

void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg){
  if(!imu_init){
    imu_init = true;    
  }
  orientation.x = msg->x;
  orientation.y = msg->y;
  orientation.z = msg->z;
  
}

void odometry_skid_steer::getEncoders(const redblade_ax2550::StampedEncoders& front_msg,
				      const redblade_ax2550::StampedEncoders& back_msg,
				      double& left_encoders, 
				      double& right_encoders,
				      double& delta_time){
  double delta_front_right_encoders = front_msg.encoders.right_wheel- prev_fr_encoder;
  double delta_front_left_encoders  = front_msg.encoders.left_wheel - prev_fl_encoder;

  double delta_back_left_encoders   = back_msg.encoders.left_wheel  - prev_br_encoder;
  double delta_back_right_encoders  = back_msg.encoders.right_wheel - prev_bl_encoder;

	   
  /* ROS_INFO("Front Right Encoder Delta %f",delta_front_right_encoders); */
  /* ROS_INFO("Back Right Encoder Delta %f",delta_back_right_encoders); */
  /* ROS_INFO("Front Left Encoder Delta %f",delta_front_left_encoders); */
  /* ROS_INFO("Back Left Encoder Delta %f",delta_back_left_encoders); */

  //Combine both wheels into "bigger" wheels such wheels
  left_encoders  = (delta_front_left_encoders  + delta_back_left_encoders)/2;
  right_encoders = (delta_front_right_encoders + delta_back_right_encoders)/2;
  ros::Time now = ros::Time::now();
  delta_time = (now - prev_time).toSec();
  //set prev_time to now so it can be used in next itertaion of loop
  
};

void odometry_skid_steer::getPosition(const redblade_ax2550::StampedEncoders& front_msg,
				      const redblade_ax2550::StampedEncoders& back_msg,
				      const geometry_msgs::Vector3& orientation_msg,
				      double& delta_time,
				      double& distance_delta,
				      double& theta_delta){

  //Is this orientation right???  Should the sin/cos be flipped?
  double dX = distance_delta*cos(theta_delta);
  double dY = distance_delta*sin(theta_delta);  

  x_pos+=dX;
  y_pos+=dY;
  theta+=theta_delta;

};
/*Get incremental time, distance and heading*/
void odometry_skid_steer::getDeltas(const redblade_ax2550::StampedEncoders& front_msg,
				    const redblade_ax2550::StampedEncoders& back_msg,
				    const geometry_msgs::Vector3& orientation_msg,
				    double& delta_time,
				    double& distance_delta,
				    double& theta_delta){
  double left_encoders,right_encoders;
  getEncoders(front_msg,back_msg,left_encoders,right_encoders,delta_time);
  distance_delta = ((left_encoders+right_encoders)/2)/(clicks_per_m);
  //compute the change in theta with the imu z gyro
  if(left_encoders==0 and right_encoders==0){
    theta_delta = 0;
  }else{
    theta_delta = (orientation.z - prev_orientation.z);
    //wrap to pi
    if(theta_delta > M_PI){
      theta_delta -= 2*M_PI;
    }else if(theta_delta < -M_PI){
      theta_delta += 2*M_PI;
    }
  }
};



nav_msgs::Odometry odometry_skid_steer::getOdometry(const redblade_ax2550::StampedEncoders& front_msg,
						    const redblade_ax2550::StampedEncoders& back_msg,
						    const geometry_msgs::Vector3& orientation_msg){
  double delta_time=0,distance_delta=0,theta_delta=0;
  geometry_msgs::Twist twist_vel;
  getDeltas(front_msg,
	    back_msg,
	    orientation_msg,
	    delta_time,
	    distance_delta,
	    theta_delta);
  getPosition(front_msg,
	      back_msg,
	      orientation_msg,
	      delta_time,
	      distance_delta,
	      theta_delta);
  getVelocities(front_msg,
		back_msg,
		theta_delta,
		twist_vel);

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
  odom.twist.twist = twist_vel;
  
  //TODO: covariance stuff lul
  odom.pose.covariance[0] = pos_cov;
  odom.pose.covariance[7] = pos_cov;
  odom.pose.covariance[14] = 1e100;
  odom.pose.covariance[21] = 1e100;
  odom.pose.covariance[28] = 1e100;
  odom.pose.covariance[35] = rot_cov;

  update(front_msg,
	 back_msg,
	 orientation_msg);
  
  return odom;
};


void odometry_skid_steer::update(const redblade_ax2550::StampedEncoders& front_msg,
				 const redblade_ax2550::StampedEncoders& back_msg,
				 const geometry_msgs::Vector3& orientation_msg){
  prev_fr_encoder  = front_msg.encoders.right_wheel;
  prev_fl_encoder  = front_msg.encoders.left_wheel;
  prev_br_encoder  = back_msg.encoders.right_wheel;
  prev_bl_encoder  = back_msg.encoders.left_wheel;
  prev_orientation.x = orientation_msg.x;
  prev_orientation.y = orientation_msg.y;
  prev_orientation.z = orientation_msg.z;
  ros::Time now = ros::Time::now();
  prev_time = now;

};


odometry_skid_steer::odometry_skid_steer(std::string odom_frame_id,
					 double rot_cov_, 
					 double pos_cov_,
					 double wheel_base_width,
					 double eff_wheel_base_width){
  odom_frame_id= odom_frame_id;
  
  x_pos = 0, y_pos = 0, theta = 0;
  x_vel = 0, y_vel = 0, theta_vel = 0;
  rot_cov = rot_cov_;
  pos_cov = pos_cov_;
  prev_fr_encoder = 0, prev_fl_encoder= 0, prev_br_encoder= 0, prev_bl_encoder= 0;
  prev_orientation.x = 0;
  prev_orientation.y = 0;
  prev_orientation.z = 0;
  wheel_base_width=wheel_base_width;
  eff_wheel_base_width=eff_wheel_base_width;
}
odometry_skid_steer::~odometry_skid_steer(){

}

void odometry_skid_steer::getVelocities(const redblade_ax2550::StampedEncoders& front_msg,
					const redblade_ax2550::StampedEncoders& back_msg,
					double theta_delta,
					geometry_msgs::Twist& twist){
  double delta_time,left_encoders,right_encoders;
  getEncoders(front_msg,back_msg,left_encoders,right_encoders,delta_time);
  double Vr = (right_encoders/clicks_per_m)/delta_time;
  double Vl = (left_encoders/clicks_per_m)/delta_time;
  double Vx = (Vr+Vl)/2;
  //double w  = (Vr-Vl)/(eff_wheel_base_width);
  double theta_v = theta_delta / delta_time;
  twist.linear.x = Vx;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = theta_v;
}


void publish_loop(odometry_skid_steer odomSS){
  //publish odom messages
  nav_msgs::Odometry odom = odomSS.getOdometry(front_encoders,back_encoders,orientation);
  odom_pub.publish(odom);  
}  


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_skid_steer");
  ros::NodeHandle n; //in the global namespace
  ros::NodeHandle nh("~");//local namespace, used for params
  std::string front_encoder_namespace,back_encoder_namespace,imu_namespace,odom_namespace;
  double rot_cov_,pos_cov_;
  double wheel_base_width,eff_wheel_base_width;
  //See odometry_skid_steer.h for all constants
  n.param("front_encoders", front_encoder_namespace, std::string("/front_encoders"));
  n.param("back_encoders", back_encoder_namespace, std::string("/back_encoders"));
  n.param("imu", imu_namespace, std::string("/imu"));
  n.param("rotation_covariance",rot_cov_, 1.0);
  n.param("position_covariance",pos_cov_, 1.0);
  n.param("odom_frame_id", odom_frame_id, std::string("odom"));
  n.param("odom", odom_frame_id, std::string("/odom"));
  n.param("wheel_base_width", wheel_base_width, 0.473);
  n.param("effective_wheel_base_width", eff_wheel_base_width, 1.0);
  odometry_skid_steer odomSS(odom_frame_id,rot_cov_,pos_cov_,wheel_base_width,eff_wheel_base_width);

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

  odom_pub = n.advertise<nav_msgs::Odometry>(odom_namespace, 10);
  
  //Set up rate for cmd_vel topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz

  //publish cmd_vel topic every 25 ms (40 hz)
  while(ros::ok()){
      if(front_recv and back_recv){
  	publish_loop(odomSS);
	front_recv = false;
	back_recv = false;
      }
    //sleep for a bit to stay at 40 hz
    cmd_vel_rate.sleep();
  }

  spinner.stop();
  return(0);
}
