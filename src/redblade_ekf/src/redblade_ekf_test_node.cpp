/*
- Suggestions
 1) Initialize the EKF using the coordinates of the field (line 244)
 2) Propagate the EKF using wheel encoders (line 102)
 3) Throw out bad gps measurements?
 4) Plot the wheel encoders over time to gage accuracy of propagation
*/

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

//Random number generator to test GPS blockage
/*
  A->A   A->B
  B->A   B->B

  A = 0 = GPS is present
  B = 1 = GPS is not present    
*/
double probs[] = {0.9,0.1,
		  0.2,0.8};
// double probs[] = {1.0,0.0,
// 		  0.25,0.75};
//int index;
struct markov_gen{
  int index;
  int random(){
    double r = ((double)rand())/(INT_MAX);
    if(index==0){index=(r<probs[0])?(0):(1);return 0;}
    if(index==1){index=(r<probs[2])?(0):(1);return 1;}
  };
  int add(){
    index++;
  };
};

markov_gen rand_gen;//Just for testing GPS blockage

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
bool hasGPS = false;
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
  hasGPS = true;
  current_gps = *gps_msg;
  //ROS_INFO("GPS");
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  if(!odom_init){
    odom_init = true;
  }else{
    current_odom = *odom_msg;
  }
}


void publish_loop(){  
  //ROS_INFO("Odom init:%d imu_init:%d",odom_init,imu_init);
  if(odom_init && imu_init){
    Vector u;
    Vector x;
    Matrix P;
    //bool blockedGPS = rand_gen.random();
    rand_gen.add();
    bool blockedGPS = false;
    //bool blockedGPS = rand_gen.index>250;
    //bool blockedGPS = (rand_gen.index%50>25 and rand_gen.index%50<50);
    //ROS_INFO("Blocked GPS:%d hasGPS:%d",blockedGPS,hasGPS);
    
    if(hasGPS and not blockedGPS){
      Vector z(5);
      z(1) = current_gps.pose.pose.position.x;
      z(2) = current_gps.pose.pose.position.y;
      z(3) = current_odom.twist.twist.linear.x;
      z(4) = current_imu.z + heading_offset*2*M_PI;
      z(5) = current_odom.twist.twist.angular.z;
      //ofstream daters("ekf_data_collect.txt",std::ofstream::app);
      daters << (ros::Time::now()).toSec() << ",";
      daters << z(1) << "," << z(2) << "," << z(3) << "," << z(4) << "," << z(5) << ",";
      //ROS_INFO("%lf, %lf, %lf, %lf, %lf", z(1), z(2), z(3), z(4), z(5));
      double test_x = (pose.x+current_odom.twist.twist.linear.x*cos( pose.theta )*0.2)+(cos(pose.theta)*0.21);
      double test_y = (pose.y+current_odom.twist.twist.linear.x*sin( pose.theta )*0.2)+(sin(pose.theta)*0.21);
      //ROS_INFO("linear velocity: %lf", z(3));
      //ROS_INFO("(%lf, %lf)\t(%lf, %lf)\t(%lf,%lf)\n",z(1),z(2),test_x,test_y,(fabs(test_x-z(1))),(fabs(test_y-z(2))));
      
      ekf.step(u, z);
      x = ekf.getX();
      hasGPS=false;
    }else{
      Vector z(5);
      double heading = pose.theta+current_odom.twist.twist.angular.z*0.2;
      z(1) = (pose.x+current_odom.twist.twist.linear.x*cos( heading )*0.2)+(cos(heading)*0.21);
      z(2) = (pose.y+current_odom.twist.twist.linear.x*sin( heading )*0.2)+(sin(heading)*0.21);
      z(3) = current_odom.twist.twist.linear.x;
      z(4) = current_imu.z + heading_offset*2*M_PI;
      z(5) = current_odom.twist.twist.angular.z;
      
      ekf.step(u,z);
      x = ekf.getX();
      //ROS_INFO("No GPS available, faking measurements");
      //x = ekf.predict(u);
    }
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
    daters << (ros::Time::now()).toNSec();
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

//method to split string by a delimiter
void split_to_double(const std::string &s, char delim, std::vector<double> &elements){
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss,item,delim)){
    elements.push_back(atof(item.c_str()));
  }
}

//convienence method so i don't have to do this over and over
void parse_file_to_vector(std::string &filename, std::vector<std::string> &lines){
  std::string item;
  std::ifstream file(filename.c_str());
  while(std::getline(file, item, '\n')){
    lines.push_back(item);
  }
}

//read in the survey points
void read_in_survey_points(std::vector<std::vector<double> > &survey_points, std::string survey_file){
  std::vector<std::string> lines;
  parse_file_to_vector(survey_file, lines);
  for(int i = 0; i < lines.size(); i++){
    std::vector<double> temp_doubles;
    split_to_double(lines[i], ',', temp_doubles);
    survey_points.push_back(temp_doubles);
  }
}

//rotate a specified point by a given angle using the
//origin as the rotation center
void rotation_matrix(std::vector<double> &point, double theta){
  double x = point[0];
  double y = point[1];
  point[0] = x*cos(theta) - y*sin(theta);
  point[1] = x*sin(theta) + y*cos(theta);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "redblade_ekf");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  
  //params
  std::string survey_file;
  bool single_i;

  //read in parameters
  nh.param("survey_file", survey_file, std::string("survey_enu.csv"));
  nh.param("single_or_triple", single_i, true);

  //publishers
  ekf_2d_pub = n.advertise<geometry_msgs::Pose2D>("redblade_ekf/2d_pose",10);
  ekf_odom_pub = n.advertise<nav_msgs::Odometry>("redblade_ekf/odom", 10);

  //open csv for writing (debug stuff)
  daters.open("/home/redblade/Documents/Redblade/ekf_data_collect.txt");
  heading_offset = 0;

  //read in survey file
  std::vector<std::vector<double> > survey_points;
  read_in_survey_points(survey_points, survey_file);
  double orientation = atan2(survey_points[1][1]-survey_points[0][1],
			     survey_points[1][0]-survey_points[0][0]);

  //initialize ekf
  const unsigned num_states = 6;
  const unsigned m = 5;//number of measures
  
  //initial covariance of estimate
  static const double _P0[] = {0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
  			       0.0, 0.04, 0.0, 0.0, 0.0, 0.0,
  			       0.0, 0.0, 0.0012, 0.0, 0.0, 0.0,
  			       0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
  			       0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
  			       0.0, 0.0, 0.0, 0.0, 0.0, 0.0012};//still gotta figure out this bias
  Matrix P0(num_states,num_states,_P0);
  
  //initial estimate, different for single i vs. triple i
  Vector x(num_states);
  std::vector<double> start_position(2,0);
  if(single_i){
    start_position[0] = 1.715;//determined by rj, jamie, and bob
    start_position[1] = 1.73;//determined by rj, jamie, and bob
    rotation_matrix(start_position, orientation);
    x(1) = start_position[0];
    x(2) = start_position[1];
    x(3) = orientation;
    x(4) = 0.0;//she ain't moving
    x(5) = 0.0;//she ain't turnin'
    x(6) = -orientation;//TODO, based on x(3)
  }else{
    ROS_INFO("Made it to the triple I initialization");
    //start_position[0] = 2.53;//determined by rj, jamie, and bob
    start_position[0] = 1.715;
    start_position[1] = 4.385;//determined by rj, jamie, and bob
    rotation_matrix(start_position, orientation);
    orientation -= (M_PI/2);
    wrapToPi(orientation);
    ROS_INFO("Orientation: %lf", orientation);
    x(1) = start_position[0];
    x(2) = start_position[1];
    x(3) = orientation;
    x(4) = 0.0;//she ain't moving
    x(5) = 0.0;//she ain't turnin'
    x(6) = -orientation;
  }

  /*static const double _P0[] = {400, 0.0, 0.0, 0.0, 0.0, 0.0,
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
  */
  //intialize ze filter
  ekf.init(x, P0);

  rand_gen.index = 0;
  ros::Subscriber imu_sub = n.subscribe ("/imu/integrated_gyros", 1, imuCallback);
  ros::Subscriber gps_sub = n.subscribe ("/gps", 1, gpsCallback);
  ros::Subscriber odom_sub = n.subscribe ("/odom", 10, odomCallback);
  ros::Rate pub_rate(5); 
  ros::AsyncSpinner spinner(3);
  spinner.start();
  while(ros::ok()) {
    publish_loop();
    pub_rate.sleep();
    //usleep(200000);
  }
  //ros::spin();
  spinner.stop();  

  return 0;
}


