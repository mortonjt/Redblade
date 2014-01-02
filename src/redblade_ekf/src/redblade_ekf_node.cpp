#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "redblade_ekf.hpp"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <tf/transform_listener.h>
//#include <boost/thread/mutex.hpp>

//#include "transform_broadcaster.cpp"

using namespace std;
using namespace Kalman;
//using namespace tf;
//using namespace ros;


geometry_msgs::Vector3 current_imu;
nav_msgs::Odometry current_gps;
nav_msgs::Odometry current_odom;
ros::Time imu_stamp;
redblade_ekf ekf;
ofstream daters;
int heading_offset;

ros::Publisher ekf_odom_pub;
nav_msgs::Odometry ekf_odom;
ros::Publisher ekf_2d_pub;
geometry_msgs::Pose2D pose;
//ros::Publisher cmd_vel_pub;
//geometry_msgs::Twist cmd_vel;

//tf::TransformBroadcaster* odom_broadcaster;

bool odom_init = false;
bool imu_init = false;
bool theta_covar_thresh_reached = false;


void wrapToPi(double &angle){
  while(angle > M_PI || angle < -M_PI){
    if(angle > M_PI){
      angle -= 2*M_PI;
    }else if(angle < -M_PI){
      angle += 2*M_PI;
    }
  }
}

void imuCallback(const geometry_msgs::Vector3::ConstPtr& imu_msg){
  if(!imu_init){
    imu_init = true;
    current_imu = *imu_msg;
  }else{
    //check for rollover stuff
    if(abs(imu_msg->z-current_imu.z) > M_PI){
      if(imu_msg->z > current_imu.z){
	heading_offset--;
      }else{
	heading_offset++;
      }
    }

    current_imu = *imu_msg;
    imu_stamp = ros::Time::now();
  }
}

void gpsCallback(const nav_msgs::Odometry::ConstPtr& gps_msg){
  
  if(odom_init && imu_init){
    Vector u;
    Vector z(5);
    Vector x;
    Matrix P;
    
    z(1) = gps_msg->pose.pose.position.x;
    z(2) = gps_msg->pose.pose.position.y;
    z(3) = current_odom.twist.twist.linear.x;
    z(4) = current_imu.z + heading_offset*2*M_PI;
    z(5) = current_odom.twist.twist.angular.z;
    //ofstream daters("ekf_data_collect.txt",std::ofstream::app);
    daters << (ros::Time::now()).toSec() << ",";
    daters << z(1) << "," << z(2) << "," << z(3) << "," << z(4) << "," << z(5) << ",";
    //ROS_INFO("%lf, %lf, %lf, %lf, %lf", z(1), z(2), z(3), z(4), z(5));
    
    ekf.step(u, z);
    x = ekf.getX();

    //construct 2D pose message and publish
    pose.x = x(1);
    pose.y = x(2);
    wrapToPi(x(3));
    pose.theta = x(3);    
    ekf_2d_pub.publish(pose);

    for(int i = 1; i < 7; i++){
      daters << x(i) << ",";
    }
    //daters << x(1) << "," << x(2) << "," << x(3) << "," << x(4) << x(5) << x(6);

    //construct Odometry message and publish
    ekf_odom.header.stamp = ros::Time::now();
    ekf_odom.header.frame_id = "odom_combined";
 
    //grab covariance from EKF
    P = ekf.calculateP();
    for(int i = 1; i < 7; i++){
      for(int j = 1; j < 7; j++){
	daters << P(i,j) << ",";
      }
    }
    daters << "\n";

    //set the position and position covariance calculated with EKF
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(x(3));
    ekf_odom.pose.pose.position.x = x(1);
    ekf_odom.pose.pose.position.y = x(2);
    ekf_odom.pose.pose.position.z = .195;
    ekf_odom.pose.pose.orientation = odom_quat;
    ekf_odom.pose.covariance[0] = P(1,1);
    ekf_odom.pose.covariance[7] = P(2,2);
    ekf_odom.pose.covariance[14] = 1e100;//not used
    ekf_odom.pose.covariance[21] = 1e100;//not used
    ekf_odom.pose.covariance[28] = 1e100;//not used
    ekf_odom.pose.covariance[35] = P(3,3);
 
    //set the velocity and velocity covariance calculated with EKF
    ekf_odom.child_frame_id = "base_footprint";
    ekf_odom.twist.twist.linear.x = x(4);
    ekf_odom.twist.twist.linear.y = 0.0;
    ekf_odom.twist.twist.angular.z = x(5);
    ekf_odom.twist.covariance[0] = P(4,4);
    ekf_odom.twist.covariance[7] = 1e100;//not used
    ekf_odom.twist.covariance[14] = 1e100;//not used
    ekf_odom.twist.covariance[21] = 1e100;//not used
    ekf_odom.twist.covariance[28] = 1e100;//not used
    ekf_odom.twist.covariance[35] = P(5,5);
    

    //publish the message
    ekf_odom_pub.publish(ekf_odom);

    //this was needed for the nav_stack, probably not needed now, scared to get rid of it yet though
    if(!theta_covar_thresh_reached){
      /*cmd_vel.linear.x = 0.75;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = 0.0;

      cmd_vel_pub.publish(cmd_vel);*/

      //if our headings covariance is less than 0.05
      if(P(3,3) < 0.05){
	theta_covar_thresh_reached = true;
	ROS_INFO("EKF theta heading initialized, publishing transform for SLAM now.\n");
	
	/*cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;
	
	cmd_vel_pub.publish(cmd_vel);*/
      }
    }else{
      //this stuff isn't even necessary, i don't know why i'm keeping it
      
      //publish the transfrom from odom_combined -> base_footprint
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom_combined";
      odom_trans.child_frame_id = "base_footprint";
      odom_trans.transform.translation.x = x(1);
      odom_trans.transform.translation.y = x(2);
      odom_trans.transform.translation.z = .195;
      odom_trans.transform.rotation = odom_quat;

    
      //publish transform
      //odom_broadcaster->sendTransform(odom_trans);
    }
  }    
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  if(!odom_init){
    odom_init = true;
  }else{
    current_odom = *odom_msg;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "redblade_ekf");
  ros::NodeHandle n;

  //odom_broadcaster = new tf::TransformBroadcaster();

  //ekf.odom_broadcaster = new tf::TransformBroadcaster();

  ekf_2d_pub = n.advertise<geometry_msgs::Pose2D>("redblade_ekf/2d_pose",10);
  ekf_odom_pub = n.advertise<nav_msgs::Odometry>("redblade_ekf/odom", 10);
  //cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 2);
  
  daters.open("/home/redblade/Documents/Redblade/ekf_data_collect.txt");
  //daters << "Test";
  heading_offset = 0;

  //initialize ekf
  const unsigned num_states = 6;
  const unsigned m = 5;//number of measures
  
  //initial covariance of estimate
  static const double _P0[] = {400, 0.0, 0.0, 0.0, 0.0, 0.0,
			     0.0, 400, 0.0, 0.0, 0.0, 0.0,
			     0.0, 0.0, pow(1.57,2), 0.0, 0.0, 0.0,
			     0.0, 0.0, 0.0, .25, 0.0, 0.0,
			     0.0, 0.0, 0.0, 0.0, 0.0625, 0.0,
			     0.0, 0.0, 0.0, 0.0, 0.0, pow(1.57,2)};
  Matrix P0(num_states,num_states,_P0);
  
  //initial estimate
  Vector x(num_states);
  x(1) = 0.0;
  x(2) = 0.0;
  x(3) = 1.57;
  x(4) = 0.0;
  x(5) = 0.0;
  x(6) = 0.0;
  //intialize ze filter
  ekf.init(x, P0);

 
  ros::Subscriber imu_sub = n.subscribe ("/imu/integrated_gyros", 1, imuCallback);
  ros::Subscriber gps_sub = n.subscribe ("/gps", 1, gpsCallback);
  ros::Subscriber odom_sub = n.subscribe ("/odom", 10, odomCallback);

  ros::spin();

  return 0;
}


  /*ros::init(argc,argv,"Arduino_RC");
  ros::NodeHandle n;
  ros::Publisher Arduino_RC_pub = n.advertise<geometry_msgs::Twist>("Arduino_RC",100);
  ros::Rate loop_rate(20); //publish at 20 hz.
  //ros::init(argc, argv, "redblade_ekf_node");
  ros::NodeHandle n;
  ros::Publisher redblade_ekf_pub_ = n.advertise<geometry_msgs::Pose2D>("redblade_ekf",5);
  ros::Rate loop_rate(5);//5 Hz

  
  while(ros::ok()) {
    //PUBLISH LINEAR & ANGULAR(RAD/SEC) VELOCITIES, BOTH INTS
    /*geometry_msgs::Twist msg;
    msg.linear.x = speedInMps;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = ((right-left)/wheelbase)*roboteqUnits/angresolution;

    Arduino_RC_pub.publish(msg);
    
    loop_rate.sleep();
    
    
  }
  
  //delete ekf;
  
  return 0;
*/
