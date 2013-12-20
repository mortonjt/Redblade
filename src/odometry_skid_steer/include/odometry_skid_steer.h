#include <ros/ros.h>
#include "ax2550/StampedEncoders.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>


class odometry_skid_steer{
 public:
  //Odometry variables
  double x_pos , y_pos , theta ;
  double x_vel , y_vel , theta_vel ;
  double rot_cov ;
  double pos_cov ;

  long prev_fr_encoder,prev_fl_encoder,prev_br_encoder,prev_bl_encoder;
  geometry_msgs::Vector3 prev_orientation;

  odometry_skid_steer(double rot_cov_, double pos_cov_);
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
