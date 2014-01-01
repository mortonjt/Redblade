#include <ros/ros.h>
#include "redblade_ax2550/StampedEncoders.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>


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

  void getVelocities(const redblade_ax2550::StampedEncoders& front_msg,
		     const redblade_ax2550::StampedEncoders& back_msg,
		     double theta_delta,
		     geometry_msgs::Twist& twist);

  void getEncoders(const redblade_ax2550::StampedEncoders& front_msg,
		   const redblade_ax2550::StampedEncoders& back_msg,
		   double& left_encoders, 
		   double& right_encoders,
		   double& delta_time);

  void getDeltas(const redblade_ax2550::StampedEncoders& front_msg,
		 const redblade_ax2550::StampedEncoders& back_msg,
		 const geometry_msgs::Vector3& orientation_msg,
		 double& delta_time,
		 double& distance_delta,
		 double& theta_delta);

  void getPosition(const redblade_ax2550::StampedEncoders& front_msg,
		   const redblade_ax2550::StampedEncoders& back_msg,
		   const geometry_msgs::Vector3& orientation_msg,
		   double& delta_time,
		   double& distance_delta,
		   double& theta_delta);

  void update(const redblade_ax2550::StampedEncoders& front_msg,
	      const redblade_ax2550::StampedEncoders& back_msg,
	      const geometry_msgs::Vector3& orientation_msg);

  nav_msgs::Odometry getOdometry(const redblade_ax2550::StampedEncoders& front_msg,
				 const redblade_ax2550::StampedEncoders& back_msg,
				 const geometry_msgs::Vector3& orientation_msg);
};



