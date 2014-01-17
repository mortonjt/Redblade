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
#include <fstream>
#include <sstream>
#include <iostream>

#define POLE_HITS 4
#define LUDICROUS_HITS 1
#define RANGE_THRESH 0.05

std::vector<std::vector<double> > survey_points;
double orientation;
std::string survey_file;

//dimensions for snow field 
double SI_snow_length_max = 13;
double SI_snow_length_min = 3;
double SI_snow_top = 2.5;
double SI_snow_bottom = 1.5;

double TI_snow_length_max = 13;
double TI_snow_length_min = 3;
double TI_snow_top = 5;
double TI_snow_bottom = 2;


geometry_msgs::Pose2D cur_pos;
sensor_msgs::LaserScan cur_scan;
ros::Publisher pole_pub;

std::ofstream lidar_file;

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

void check_for_pole(){
  int num_scans = (int)((cur_scan.angle_max-cur_scan.angle_min)/cur_scan.angle_increment);

  int hit_start = -1;
  int hit_start_range = -1;
  int hit_count = 0;
  int lud_count = 0;

  //loop through each scan
  for(int i = 0; i < num_scans; i++){
    if(hit_start == -1){
      if(cur_scan.ranges[i]==INFINITY){
	continue;
      }else if(std::isnan(cur_scan.ranges[i])){
	continue;
      }else{
	hit_start_range = cur_scan.ranges[i];
	hit_count++;
	hit_start = i;
	lud_count = 0;//may need to be moved
      }
    }else{
      //check for invalid
      if((cur_scan.ranges[i]==INFINITY) ||
	 (std::isnan(cur_scan.ranges[i])) ||
	 (fabs(cur_scan.ranges[i]-hit_start_range) > RANGE_THRESH)){
	if(lud_count < LUDICROUS_HITS){
	  lud_count++;
	}else{
	  if(hit_count < POLE_HITS){
	    //nothing found, reset
	    hit_start = -1;
	    hit_start_range = -1;
	    hit_count = 0;
	    lud_count = 0;
	  }else{
	    //something was found, save her
	  }
	}
      }else{
	//must be valid, add to 
      }
    }
  }

  
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  cur_scan = *scan_msg;
  int num_scans = (int)((cur_scan.angle_max-cur_scan.angle_min)/cur_scan.angle_increment);

  //check to make sure i understand laser messages
  ROS_INFO("Num scans: %d",num_scans);
  ROS_INFO("Angle min: %f\tAngle max:%f",cur_scan.angle_min, cur_scan.angle_max);

  /*if(cur_scan.ranges[0]==INFINITY){
    ROS_INFO("She's infinite boi");
  }else if(std::isnan(cur_scan.ranges[0])){
    ROS_INFO("She's nan boi");
  }else{
    ROS_INFO("She's valid boi");
    }*/
  
  lidar_file << cur_pos.x << "," << cur_pos.y << "," << cur_pos.theta << ",";

  for(int i = 0; i < num_scans; i++){
    if(i == (num_scans-1)){
      lidar_file << cur_scan.ranges[i] << "\n";
    }else{
      lidar_file << cur_scan.ranges[i] << ",";
    }
  }
  
  //check_for_pole();

}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  cur_pos = *pose_msg;
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
void read_in_survey_points(){
  std::vector<std::string> lines;
  parse_file_to_vector(survey_file, lines);
  for(int i = 0; i < lines.size(); i++){
    std::vector<double> temp_doubles;
    split_to_double(lines[i], ',', temp_doubles);
    survey_points.push_back(temp_doubles);
  }
}

//use two survey points to find the angle of the field with respect
//to an ENU coordinate frame
void get_orientation(){
  orientation = atan2(survey_points[1][1]-survey_points[0][1],
		      survey_points[1][0]-survey_points[0][0]);
}

//rotate a specified point by a given angle using the
//origin as the rotation center
void rotation_matrix(geometry_msgs::Pose2D& point, double theta){
  double x = point.x;
  double y = point.y;
  point.x = x*cos(theta) - y*sin(theta);
  point.y = x*sin(theta) + y*cos(theta);
}



bool checkBoundaries(double range, double laser_theta){

  //put robot position in frame where bottom is aligned with 0deg.
  rotation_matrix(curPos,-orientation);

  geometry_msgs::Pose2D laserPoint;

  laserPoint.x = curPos.x + range*cos(laser_theta+curPos.theta);
  laserPoint.y = curPos.y + range*sin(laser_theta+curPos.theta);

  if(single_i){
    if(laserPoint.x > SI_snow_length_max || laserPoint.x < SI_snow_length_min)
      return false;
    if(laserPoint.y > SI_snow_top || laserPoint.y < SI_snow_bottom)
      return false;
  }else{    
    if(laserPoint.x TI_snow_length_max || laserPoint.x < TI_snow_length_min)
      return false;
    if(laserPoint.y > TI_snow_top || laserPoint.y < TI_snow_bottom)
      return false;
  }

  return true;
}

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "chad_laser_node");
  ros::NodeHandle n;//global
  ros::NodeHandle nh("~");//local

  std::string pose_namespace, laser_namespace, pole_namespace;

  //read in params
  nh.param("ekf_namespace",pose_namespace,std::string("/redblade_ekf/2d_pose"));
  nh.param("laser_namespace",laser_namespace,std::string("/scan"));
  nh.param("pole_namespace",pole_namespace,std::string("/lidar/pole"));
  nh.param("survey_file",survey_file,std::string("/home/redblade/Documents/Redblade/config/survey_enu.csv"));
  nh.param("single_or_triple",single_i,true);
  
  //Subscribe to the EKF topic
  ros::Subscriber pose_sub = n.subscribe(pose_namespace, 1, poseCallback);

  //Subscribe to laser scan topic
  ros::Subscriber scan_pub = n.subscribe<sensor_msgs::LaserScan> (laser_namespace, 1, scanCallback);

  //Set up publisher for pole point
  pole_pub = n.advertise<geometry_msgs::PointStamped>(pole_namespace, 1);

  lidar_file.open("/home/redblade/Documents/Redblade/lidar_collect.csv");
  
  read_in_survey_points();
  get_orientation();


ros::AsyncSpinner spinner(2);
  spinner.start();
  while(ros::ok()){
    //we can do other stuff in here if we need to
    usleep(50000);
  }
  spinner.stop();
}
