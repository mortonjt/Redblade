#include <ros/ros.h>
#include "ax2550/StampedEncoders.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>


static double clicks_per_m = 15768.6;

class odometry_skid_steer{
 public:

  //Constants
  double wheel_base_width;
  double eff_wheel_base_width;

  //Odometry variables
  double x_pos , y_pos , theta ;
  double x_vel , y_vel , theta_vel ;
  double rot_cov ;
  double pos_cov ;
  std::string odom_frame_id;
  long prev_fr_encoder,prev_fl_encoder,prev_br_encoder,prev_bl_encoder;
  geometry_msgs::Vector3 prev_orientation;

  odometry_skid_steer(std::string odom_frame_id, double rot_cov_, double pos_cov_,double wheel_base_width);
  odometry_skid_steer(std::string odom_frame_id, double rot_cov_, double pos_cov_,double wheel_base_width,double eff_wheel_base_width);
  ~odometry_skid_steer();

  void getVelocities(const ax2550::StampedEncoders& front_msg,
		     const ax2550::StampedEncoders& back_msg,
		     geometry_msgs::Twist& twist);

  void getEncoders(const ax2550::StampedEncoders& front_msg,
		   const ax2550::StampedEncoders& back_msg,
		   double& delta_time,
		   double& left_encoders, 
		   double& right_encoders);

  void getDeltas(const ax2550::StampedEncoders& front_msg,
		 const ax2550::StampedEncoders& back_msg,
		 const geometry_msgs::Vector3& orientation_msg,
		 double& delta_time,
		 double& distance_delta,
		 double& theta_delta);

  void getPosition(const ax2550::StampedEncoders& front_msg,
		   const ax2550::StampedEncoders& back_msg,
		   const geometry_msgs::Vector3& orientation_msg,
		   double& delta_time,
		   double& distance_delta,
		   double& theta_delta);

  void update(const ax2550::StampedEncoders& front_msg,
	      const ax2550::StampedEncoders& back_msg,
	      const geometry_msgs::Vector3& orientation_msg,
	      double delta_time,
	      double distance_delta,
	      double theta_delta);

  nav_msgs::Odometry getOdometry(const ax2550::StampedEncoders& front_msg,
				 const ax2550::StampedEncoders& back_msg,
				 const geometry_msgs::Vector3& orientation_msg);
};


void odometry_skid_steer::getEncoders(const ax2550::StampedEncoders& front_msg,
				      const ax2550::StampedEncoders& back_msg,
				      double& delta_time,
				      double& left_encoders, 
				      double& right_encoders){
  double delta_front_right_encoders = -1 * (front_msg.encoders.right_wheel - prev_fr_encoder);
  double delta_front_left_encoders = front_msg.encoders.left_wheel-prev_fl_encoder;

  double delta_back_left_encoders =  back_msg.encoders.left_wheel - prev_br_encoder;
  double delta_back_right_encoders = -1 * (back_msg.encoders.right_wheel - prev_bl_encoder);

  ROS_INFO("Front Right Encoder Delta %f",delta_front_right_encoders);
  ROS_INFO("Front Left Encoder Delta %f",delta_front_left_encoders);
  ROS_INFO("Back Right Encoder Delta %f",delta_back_right_encoders);
  ROS_INFO("Back Left Encoder Delta %f",delta_back_left_encoders);

  //Combine both wheels into "bigger" wheels such wheels
  left_encoders  = (delta_front_left_encoders  + delta_back_left_encoders)/2;
  right_encoders = (delta_front_right_encoders + delta_back_right_encoders)/2;
  
};

void odometry_skid_steer::getPosition(const ax2550::StampedEncoders& front_msg,
				      const ax2550::StampedEncoders& back_msg,
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
void odometry_skid_steer::getDeltas(const ax2550::StampedEncoders& front_msg,
				    const ax2550::StampedEncoders& back_msg,
				    const geometry_msgs::Vector3& orientation_msg,
				    double& delta_time,
				    double& distance_delta,
				    double& theta_delta){
  double left_encoders,right_encoders;
  getEncoders(front_msg,back_msg,left_encoders,right_encoders,delta_time);
  distance_delta = ((left_encoders+right_encoders)/2)/(clicks_per_m);
  theta_delta = orientation_msg.z - prev_orientation.z;
  //theta_delta = (left_encoders-right_encoders)/wheel_base_width;
};



nav_msgs::Odometry odometry_skid_steer::getOdometry(const ax2550::StampedEncoders& front_msg,
						    const ax2550::StampedEncoders& back_msg,
						    const geometry_msgs::Vector3& orientation_msg){
  double delta_time,distance_delta,theta_delta;
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
	 orientation_msg,
	 delta_time,
	 distance_delta,
	 theta_delta);
  
  return odom;
};


void odometry_skid_steer::update(const ax2550::StampedEncoders& front_msg,
				 const ax2550::StampedEncoders& back_msg,
				 const geometry_msgs::Vector3& orientation_msg,
				 double delta_time,
				 double distance_delta,
				 double theta_delta){
  prev_fr_encoder  = front_msg.encoders.right_wheel;
  prev_fl_encoder  = front_msg.encoders.left_wheel;
  prev_br_encoder  = back_msg.encoders.right_wheel;
  prev_bl_encoder  = back_msg.encoders.left_wheel;
  prev_orientation.x = orientation_msg.x;
  prev_orientation.y = orientation_msg.y;
  prev_orientation.z = orientation_msg.z;
};
