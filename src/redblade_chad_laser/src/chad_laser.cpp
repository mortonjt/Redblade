#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <cmath>

geometry_msgs::Pose2D cur_pos;
sensor_msgs::LaserScan cur_scan;
ros::Publisher pole_pub;

//keeps angles between -pi and pi so i don't have to
void wrap_pi(double &angle){
  while(angle > M_PI || angle < -M_PI){
    if(angle > M_PI){
      angle -= 2*M_PI;
    }else if(angle < -M_PI){
      angle += 2*M_PI;
    }
  }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  cur_scan = *scan_msg;

  //TODO:check for pole and publish
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  cur_pos = *pose_msg;
}

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "chad_laser_node");
  ros::NodeHandle n;//global
  ros::NodeHandle nh("~");//local

  std::string pose_namespace, laser_namespace, pole_namespace;

  //read in params
  //nh.param("blah", blah, 0.0);
  nh.param("ekf_namespace",pose_namespace,std::string("/redblade_ekf/2d_pose"));
  nh.param("laser_namespace",laser_namespace,std::string("/scan"));
  nh.param("pole_namespace",pole_namespace,std::string("lidar/pole"));

  //Subscribe to the EKF topic
  ros::Subscriber pose_sub = n.subscribe(pose_namespace, 1, poseCallback);

  //Subscribe to laser scan topic
  ros::Subscriber scan_pub = n.subscribe<sensor_msgs::LaserScan> (laser_namespace, 1, scanCallback);

  //Set up publisher for pole point
  pole_pub = n.advertise<geometry_msgs::PointStamped>(pole_namespace, 1);

ros::AsyncSpinner spinner(2);
  spinner.start();
  while(ros::ok()){
    //we can do other stuff in here if we need to
    usleep(50000);
  }
  spinner.stop();
}
