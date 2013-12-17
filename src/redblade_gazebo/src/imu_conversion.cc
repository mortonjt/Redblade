#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <cmath>

#include <assert.h>
#include <math.h>
#include <iostream>

#include <boost/format.hpp>

#include "ros/time.h"
#include "self_test/self_test.h"


geometry_msgs::Vector3 current_imu;

sensor_msgs::Imu reading;
geometry_msgs::Vector3 integrated_gyros;//bob
ros::Publisher integrated_gyro_pub;
ros::Subscriber sub;

double x_orientation_;
double y_orientation_;
double z_orientation_;

/*method added by bob*/
double wrapToPi(double angle) {
  angle += M_PI;
  bool is_neg = (angle < 0);
  angle = fmod(angle, (2.0*M_PI));
  if (is_neg) {
    angle += (2.0*M_PI);
  }
  angle -= M_PI;
  return angle;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr& data){
  
  //sensor_msgs::Imu data = *imu_msg;

  x_orientation_ = wrapToPi(x_orientation_ + data->angular_velocity.y * .01);
  y_orientation_ = wrapToPi(y_orientation_ - data->angular_velocity.x * .01);
  z_orientation_ = wrapToPi(z_orientation_ - data->angular_velocity.z * .01);
  
  integrated_gyros.x = x_orientation_;
  integrated_gyros.y = y_orientation_;
  integrated_gyros.z = z_orientation_;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_conversion");
  ros::NodeHandle n; //global namespace
  ros::NodeHandle nh("~");//local namespace, used for params

  x_orientation_ = y_orientation_ = z_orientation_ = 0; //initialize all orientations
  std::string imu_namespace,imu_integrated_namespace;

  nh.param("imu_orientation",imu_namespace,std::string("/imu"));
  nh.param("imu_integrated_gyros",imu_integrated_namespace,std::string("/imu/integrated_gyros"));

  //Setup imu subscriber
  sub = nh.subscribe(imu_namespace, 1, imuCallback);
  
  //Set up integrated gyro publisher
  integrated_gyro_pub = n.advertise<geometry_msgs::Vector3>(imu_integrated_namespace, 10);
  
  //Set up rate for integrated_gyro_pub topic to be published at
  ros::Rate integrated_gyro_rate(30);//Hz
  
  while(ros::ok()){
    integrated_gyro_pub.publish(integrated_gyros);
    integrated_gyro_rate.sleep();
    ros::spinOnce();
  }

  return(0);
}
