//#include <ros/ros.h>
#include <string>
#include <cmath>
//#include "ax2550/StampedEncoders.h"
#include "odometry_skid_steer.h"

//Constants
// static double wheel_circumference = 0.0;
// static double wheel_diameter = 0.0;
// static double wheel_base_width = 0.473;
// static double eff_wheel_base_width;
// static double wheel_base_length;
static double clicks_per_m = 15768.6;

void odometry_skid_steer::getEncoders(const redblade_ax2550::StampedEncoders& front_msg,
				      const redblade_ax2550::StampedEncoders& back_msg,
				      double& left_encoders, 
				      double& right_encoders,
				      double& delta_time){
  double delta_front_right_encoders = front_msg.encoders.right_wheel- prev_fr_encoder;
  double delta_front_left_encoders  = front_msg.encoders.left_wheel - prev_fl_encoder;
  double delta_back_right_encoders  = back_msg.encoders.right_wheel - prev_br_encoder;
  double delta_back_left_encoders   = back_msg.encoders.left_wheel  - prev_bl_encoder;
  delta_time = front_msg.encoders.time_delta;

  if(abs(delta_front_right_encoders) > 15000 or
     abs(delta_front_left_encoders ) > 15000 or
     abs(delta_back_left_encoders  ) > 15000 or
     abs(delta_back_right_encoders ) > 15000){

  
    ROS_WARN("Change in encoder values too large, ignoring these\n\tEncoder front right: %lf\n\tEncoder front left: %lf\n\t\n\tEncoder back left: %lf\n\tEncoder back right: %lf\n\tdt: %f\n",
  	     delta_front_right_encoders,
  	     delta_front_left_encoders, 
  	     delta_back_left_encoders,  
  	     delta_back_right_encoders, 
  	     delta_time);
    delta_front_right_encoders = 0;
    delta_front_left_encoders  = 0;
    delta_back_left_encoders   = 0;
    delta_back_right_encoders  = 0;
  }	   
  /* ROS_INFO("Front Right Encoder Delta %f",delta_front_right_encoders); */
  /* ROS_INFO("Back Right Encoder Delta %f",delta_back_right_encoders); */
  /* ROS_INFO("Front Left Encoder Delta %f",delta_front_left_encoders); */
  /* ROS_INFO("Back Left Encoder Delta %f",delta_back_left_encoders); */

  //Combine both wheels into "bigger" wheels such wheels
  left_encoders  = (delta_front_left_encoders  + delta_back_left_encoders)/2.0;
  right_encoders = (delta_front_right_encoders + delta_back_right_encoders)/2.0;
  
  
  //ROS_INFO("Delta time %f",delta_time);
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
    theta_delta = (orientation_msg.z - prev_orientation.z);
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


  // ROS_INFO("Cur fr %ld  Prev fr %ld",front_msg.encoders.right_wheel,prev_fr_encoder);
  // ROS_INFO("Cur fl %ld  Prev fl %ld",front_msg.encoders.left_wheel ,prev_fl_encoder);
  // ROS_INFO("Cur br %ld  Prev br %ld",back_msg.encoders.right_wheel ,prev_br_encoder);
  // ROS_INFO("Cur bl %ld  Prev bl %ld",back_msg.encoders.left_wheel  ,prev_bl_encoder);

  prev_fr_encoder  = front_msg.encoders.right_wheel;
  prev_fl_encoder  = front_msg.encoders.left_wheel;
  prev_br_encoder  = back_msg.encoders.right_wheel;
  prev_bl_encoder  = back_msg.encoders.left_wheel;   
  
  prev_orientation.x = orientation_msg.x;
  prev_orientation.y = orientation_msg.y;
  prev_orientation.z = orientation_msg.z;

};


odometry_skid_steer::odometry_skid_steer(std::string odom_frame_id,
					 double rot_cov_, 
					 double pos_cov_,
					 double wheel_base_width){
					 //double eff_wheel_base_width){
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
  //eff_wheel_base_width=eff_wheel_base_width;
}
odometry_skid_steer::~odometry_skid_steer(){

}

void odometry_skid_steer::getVelocities(const redblade_ax2550::StampedEncoders& front_msg,
					const redblade_ax2550::StampedEncoders& back_msg,
					double theta_delta,
					geometry_msgs::Twist& twist){
  double delta_time=0,left_encoders=0,right_encoders=0;
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
