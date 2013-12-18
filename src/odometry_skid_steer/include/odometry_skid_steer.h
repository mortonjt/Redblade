#include <ros/ros.h>
#include "ax2550/StampedEncoders.h"
#include <nav_msgs/Odometry.h>


//Constants
static double wheel_circumference = 0.0;
static double wheel_diameter = 0.0;
static double clicks_per_m = 15768.6;
static double wheel_base_width = 0.473;
static double wheel_base_length;

class odometry_skid_steer{
 public:
  //Odometry variables
  double x_pos , y_pos , theta ;
  double x_vel , y_vel , theta_vel ;
  double rot_cov ;
  double pos_cov ;

  odometry_skid_steer(double rot_cov_, double pos_cov_);
  ~odometry_skid_steer();

  static void getDeltaAnglePos(const ax2550::StampedEncoders& front_msg,
			       const ax2550::StampedEncoders& back_msg,
			       double& delta_time,
			       double& distance_delta,
			       double& theta_delta);

  void update(double delta_time,
	      double distance_delta,
	      double theta_delta);
  nav_msgs::Odometry getOdometry();
};
