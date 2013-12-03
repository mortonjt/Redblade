#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include "snowplow_pid/request_next_waypoints.h"
#include <string>
#include <cmath>

//Parameters that will be read in at runtime
double FAST_SPEED, SLOW_SPEED, KP, KI, KD, KP_SLOW, KI_SLOW, KD_SLOW;

//global stuff
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist vel_targets;
geometry_msgs::Vector3 current_imu;
nav_msgs::Odometry current_gps;
bool imu_init = false;
ros::ServiceClient waypoint_client;

//current waypoint stuff
geometry_msgs::Pose2D start;
geometry_msgs::Pose2D dest;
bool forward; //whether or not we go forwards or backwards to this waypoint

//global stuff for pid controller
double previous_error;
double sum_of_errors;
double total_num_of_errors;
double error;
double linear_vel;


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

double get_i_correction(double error){
  sum_of_errors += error;
  return sum_of_errors/total_num_of_errors;
}

double get_d_correction(double error){
  double d_corr = error - previous_error;
  previous_error = error;
  return d_corr;
}

/*returns the distance from end point with some magic sprinkled in (no, i will
not even attempt to explain the black magic that this method does. i blame Ryan
Wolfarth for the lack of comments. */
double distance_to_goal(geometry_msgs::Pose2D &dest, geometry_msgs::Pose2D &start){
  double x1, y1, x2, y2, x, y;
  double mS, mD, bD, quad_correct, d;
  
  x = current_gps.pose.pose.position.x;
  y = current_gps.pose.pose.position.y;
  
  //handling zero slope
  if(dest.y-start.y == 0){
    if(dest.x > start.x){
      d = dest.x - x;
    }else{
      d = x - dest.x;
    }
  }else if(dest.x-start.x == 0){//handling undef. slope
    if(dest.y > start.y){
      d = dest.y - y;
    }else{
      d = y - dest.y;
    }
  }else{//handle all other cases with non-zero, defined slope
    //convert to local reference frame: current point is origin
    x1 = start.x - x;
    y1 = start.y - y;
    x2 = dest.x - x;
    y2 = dest.y - y;
    x = 0; y = 0;

    //calculate slope and equation of perpendicular line
    mS = (y1-y2) / (x1-x2);
    mD = -1/mS;
    bD = y2 - (mD*x2);

    quad_correct = atan2(y2-y1,x2-x1);
    if(quad_correct < 0){
      quad_correct = -1;
    }else{
      quad_correct = 1;
    }
    
    d = quad_correct * (((x*mD) - y + bD) / sqrt(pow(mD,2) + 1.0));
  }

  return d;
}

//returns true when destination is reached
bool ye_ol_pid(){
  //local variables
  double desired_heading, kp_corr, ki_corr, kd_corr, pid, distance;
  double current_heading = current_imu.z;

  //TODO: implement a check for stuck method here
  
  //if this is the first time this method has been called, let's just send her in a straight line
  //for a very short peiod of time
  if(vel_targets.linear.x == 0){
    vel_targets.linear.x = FAST_SPEED * (forward?(1):(-1));// m/s
    vel_targets.angular.z = 0;//straight line, no turnin
    return false;//we ain't done yet
  }
  
  //calculate error
  desired_heading = atan2(dest.y-current_gps.pose.pose.position.y,
			  dest.x-current_gps.pose.pose.position.x);
  wrap_pi(desired_heading);
  if(!forward){
    current_heading -= M_PI;
    wrap_pi(current_heading);
  }
  error = current_heading - desired_heading;
  wrap_pi(error);

  //calculate p, i, and d correction factors
  total_num_of_errors += 1;
  if(linear_vel == FAST_SPEED){
    kp_corr = KP * error;
    ki_corr = KI * get_i_correction(error);
    kd_corr = KD * get_d_correction(error);
  }else{
    kp_corr = KP_SLOW * error;
    ki_corr = KI_SLOW * get_i_correction(error);
    kd_corr = KD_SLOW * get_d_correction(error);
  }
  pid = kp_corr + ki_corr + kd_corr;

  //set upper limit for angular velocity
  //TODO: i actually have no idea what this number should be, gonna need to figure that out
  if(pid > 0.5){
    pid = 0.5;
  }else if(pid < -0.5){
    pid = -0.5;
  }

  //check to see if we've reached our destination
  distance = distance_to_goal(dest, start);
  if(distance < 0){
    //set desired linear and angular velocities
    vel_targets.linear.x = 0;
    vel_targets.angular.z = 0;
    return true;
  }else if(distance < .25){
    vel_targets.linear.x = linear_vel * (forward?(1):(-1));
    vel_targets.angular.z = 0;
  }else{
    //set desired linear and angular velocities
    vel_targets.linear.x = linear_vel * (forward?(1):(-1));
    vel_targets.angular.z = pid;
  }
  
  //change linear velocity once we are close to the point
  if(distance < 0.75){
    linear_vel = SLOW_SPEED;
  }

  return false;
}

//Callback for the imu
void imuCallback(const geometry_msgs::Vector3::ConstPtr& imu_msg){
  if(!imu_init){
    imu_init = true;
  }
  current_imu = *imu_msg;
}

void gpsCallback(const nav_msgs::Odometry::ConstPtr& gps_msg){
  if(imu_init){//if the imu isn't publishing yet, we are just gonna ignore this message
    //copy this info
    current_gps = *gps_msg;

    //do the ol pid dance
    if(ye_ol_pid()){
      usleep(100000);
      //TODO: use service to grab the next waypoint
      snowplow_pid::request_next_waypoints srv;
      if(waypoint_client.call(srv)){
	start = srv.response.start;
	dest = srv.response.dest;
	forward = srv.response.forward;
      }else{
	ROS_ERROR("Failed to call service request_next_waypoints");
      }

      //TODO: make robot turn here to face the next waypoint

      //reinitialize all errors to zero
      previous_error = 0;
      sum_of_errors = 0;
      total_num_of_errors = 0;
      error = 0;
      linear_vel = FAST_SPEED;
    }
  }
}

//This method is called every 25 ms and will publish a Twist message for the robot
void publish_loop(){
  cmd_vel_pub.publish(vel_targets);
}

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "snowplow_pid_node");
  ros::NodeHandle n;//global namespace
  ros::NodeHandle nh("~");//local namespace, used for params
  
  std::string gps_namespace,imu_namespace;

  //read in pid parameters
  nh.param("FAST_SPEED", FAST_SPEED, 0.0);
  nh.param("SLOW_SPEED", SLOW_SPEED, 0.0);
  nh.param("KP", KP, 0.0);
  nh.param("KI", KI, 0.0);
  nh.param("KD", KD, 0.0);
  nh.param("KP_SLOW", KP_SLOW, 0.0);
  nh.param("KI_SLOW", KI_SLOW, 0.0);
  nh.param("KD_SLOW", KD_SLOW, 0.0);
  nh.param("gps",gps_namespace,std::string("/gps"));
  nh.param("imu",imu_namespace,std::string("/imu/integrated_gyros"));
  ROS_INFO("FAST: %f\tSLOW: %f\tKP: %f\t", FAST_SPEED, SLOW_SPEED, KP);
  ROS_INFO("gps_namespace: %s\t imu_namespace: %s", gps_namespace.c_str(), imu_namespace.c_str());


  //Start spinner so that callbacks happen in a seperate thread
  //~~~~~~~~~~HEY BOP, are you trying to give each callback a dedicated thread?
  //because I don't think that's what's happening here.
  ros::AsyncSpinner spinner(2);//2 threads
  spinner.start();

  //Subscribe to GPS topic
  ros::Subscriber gps_sub = n.subscribe(gps_namespace, 1, gpsCallback);

  //Subscribe to IMU topic
  ros::Subscriber imu_sub = n.subscribe(imu_namespace, 1, imuCallback);

  //set up service to grab waypoints
  waypoint_client = n.serviceClient<snowplow_pid::request_next_waypoints>("request_next_waypoints");

  //Set up cmd_vel publisher
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  //Set up rate for cmd_vel_pub topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz


  //initialize Twist messages to zeros, might not be necessary, but YOLO
  vel_targets.linear.x = 0;
  vel_targets.linear.y = 0;
  vel_targets.linear.z = 0;
  vel_targets.angular.x = 0;
  vel_targets.angular.y = 0;
  vel_targets.angular.z = 0;

  //initialize pid errors to zero
  previous_error = 0;
  sum_of_errors = 0;
  total_num_of_errors = 0;
  error = 0;
  linear_vel = FAST_SPEED;
 
  while(ros::ok()){
    publish_loop();
    
    cmd_vel_rate.sleep();
  }
  
  //Sttaahhhhhhp
  spinner.stop();
}
