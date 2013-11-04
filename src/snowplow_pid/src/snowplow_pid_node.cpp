#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <string>
#include <cmath>

//global stuff
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist vel_targets;
geometry_msgs::Vector3 current_imu;
nav_msgs::Odometry current_gps;
geometry_msgs::Pose2D start;
geometry_msgs::Pose2D dest;
bool forward; //whether or not we go forwards or backwards to this waypoint
bool imu_init = false;
//global stuff for pid controller
double previous_error;
double sum_of_errors;
double total_num_of_errors;
double error;


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

bool ye_ol_pid(){
  //local variables
  double desired_heading;
  double current_heading = current_imu.z;

  //TODO: implement a check for stuck method here
  
  //if this is the first time this method has been called, let's just send her in a straight line
  //for a very short peiod of time
  if(vel_targets.linear.x == 0){
    vel_targets.linear.x = (forward?(0.75):(-0.75));// m/s
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
      //TODO: use service to grab the next waypoint

      //reinitialize all errors to zero
      previous_error = 0;
      sum_of_errors = 0;
      total_num_of_errors = 0;
      error = 0;
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
  ros::NodeHandle n;

  //Start spinner so that callbacks happen in a seperate thread
  ros::AsyncSpinner spinner(2);//2 threads
  spinner.start();

  //Subscribe to GPS topic
  ros::Subscriber imu_sub = n.subscribe("/imu/integrated_gyros", 1, imuCallback);
  ros::Subscriber gps_sub = n.subscribe("/gps", 1, gpsCallback);

  //Subscribe to IMU topic
  //TODO

  //Set up cmd_vel publisher
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  //Set up rate for cmd_vel_pub topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz

  //initialize Tiwst messages to zeros, might not be necessary, but YOLO
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
 
  while(ros::ok()){
    publish_loop();
    
    cmd_vel_rate.sleep();
  }
  
  //Sttaahhhhhhp
  spinner.stop();
}
