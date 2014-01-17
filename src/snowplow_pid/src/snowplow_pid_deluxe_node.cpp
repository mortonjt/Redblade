
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <cmath>

#include "waypoint_management.h"

//Parameters that will be read in at runtime
double FAST_SPEED, SLOW_SPEED, KP, KI, KD, KP_SLOW, KI_SLOW, KD_SLOW;

//global stuff
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist vel_targets;
geometry_msgs::Pose2D cur_pos;
bool imu_init = false;
//ros::ServiceClient waypoint_client;

//current waypoint stuff
// geometry_msgs::Pose2D start;
// geometry_msgs::Pose2D dest;
//bool forward; //whether or not we go forwards or backwards to this waypoint

//global stuff for pid controller
double previous_error;
double sum_of_errors;
double total_num_of_errors;
double error;
double linear_vel;
bool forward_or_turn;//1 forward
bool avoidance_active;
int avoidance_counter;

Waypoint avoid_enter, avoid_exit, avoid_thin, avoid_fat;
//Waypoint start, dest;
Waypoint pole_est;

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

//Turning method
bool turn_to_heading(){
  ROS_INFO("Start Turning to Correct Heading");
  double desired_heading = atan2(dest.y-start.y,
				 dest.x-start.x);
  
  wrap_pi(desired_heading);
  if(!forward){
    desired_heading -= M_PI;
    wrap_pi(desired_heading);
  }
  
  usleep(50000);
  double error = desired_heading - cur_pos.theta;
  wrap_pi(error);

  if(error > 0){
    vel_targets.linear.x = 0;
    vel_targets.angular.z = 0.3;
  }else{
    vel_targets.linear.x = 0;
    vel_targets.angular.z = -0.3;
  }

  ROS_INFO("Desired heading %lf Current Heading %lf Error %lf",desired_heading,cur_pos.theta,error);

  // iterate until within a certain threshold
  //TODO: make this threshold a parameter
  if(fabs(error) < 0.15){
    //fairly certain I can delete part this next line that gives warning. looks like it was just for debugging abs/fabs....
    ROS_INFO("Final Error: %lf fabs(error) %lf abs(error) %lf",error,fabs(error),abs(error));
    vel_targets.linear.x = 0;
    vel_targets.angular.z = 0;
    return true;
  }
  return false;
}


/*returns the distance from end point with some magic sprinkled in (no, i will
not even attempt to explain the black magic that this method does. i blame Ryan
Wolfarth for the lack of comments. 
*/
double distance_to_goal(){
  double x1, y1, x2, y2, x, y;
  double mS, mD, bD, quad_correct, d;

  //set current position
  x = cur_pos.x;
  y = cur_pos.y;
  
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
  ROS_INFO("Ye Old Pid");
  //local variables
  double desired_heading, kp_corr, ki_corr, kd_corr, pid, distance;
  //double current_heading = current_imu.z;
  double current_heading = cur_pos.theta;

  //geometry_msgs::Pose2D cur_pos;
  //cur_pos.x = current_pose.x;
  //cur_pos.y = current_pose.y;
  
  // cur_pos.x = current_gps.pose.pose.position.x;
  // cur_pos.y = current_gps.pose.pose.position.y;

  //TODO: implement a check for stuck method here
  
  /*if this is the first time this method has been called, let's just send her in a straight line
    for a very short peiod of time*/
  if(vel_targets.linear.x == 0){
    vel_targets.linear.x = FAST_SPEED * (forward?(1):(-1));// m/s
    vel_targets.angular.z = 0;//straight line, no turnin
    usleep(100000);
    return false;//we ain't done yet
  }
  
  //calculate error
  // desired_heading = M_PI/2-atan2(dest.y-cur_pos.y,
  // 			       dest.x-cur_pos.x);
  desired_heading = atan2(dest.y-cur_pos.y,
			  dest.x-cur_pos.x);
  wrap_pi(desired_heading);
  if(!forward){
    current_heading -= M_PI;
    wrap_pi(current_heading);
  }
  error = desired_heading - current_heading;
  wrap_pi(error);
  /*ROS_INFO("Error %f",error);
    ROS_INFO("Current Heading %f",current_heading);
    ROS_INFO("Desired Heading %f",desired_heading);*/

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
  //ROS_INFO("PID %f",pid);

  if(pid > 0.5){
    pid = 0.5;
  }else if(pid < -0.5){
    pid = -0.5;
  }
  //ROS_INFO("PID %f",pid);

  //check to see if we've reached our destination
  distance = distance_to_goal();
  /*ROS_INFO("Distance %f",distance);
  ROS_INFO("Current (%f,%f) Dest (%f,%f)",
	   cur_pos.x,cur_pos.y,
	   dest.x,dest.y);*/
  ROS_INFO("Error: %f\tCurrent Heading: %f\t Desired Heading: %f",(error*(180/M_PI)),(current_heading*(180/M_PI)),(desired_heading*(180/M_PI)));
  ROS_INFO("PID: %f\tDistance: %f\t Current (%f, %f), Desination (%f, %f)\n",pid,distance,cur_pos.x,cur_pos.y,dest.x,dest.y);

  
  if(distance < 0.1){
    //set desired linear and angular velocities
    vel_targets.linear.x = 0;
    vel_targets.angular.z = 0;
    ROS_INFO("Reached Goal!!!");
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
  //ROS_INFO("vel_targets linear: %f",vel_targets.linear.x);
  //ROS_INFO("vel_targets angular %f",vel_targets.angular.z);

  return false;
}

void poleCallback(const geometry_msgs::Point::ConstPtr& pole_msg){
  pole_est.x = pole_msg->x;  
  pole_est.y = pole_msg->y;
  //this probably isn't how it's being passed
  
  bool pole_in_path = false;
  if(avoidance_counter == 0 || !avoidance_active){
    pole_in_path =  checkForPole(avoid_enter, avoid_exit, 
				    avoid_thin, avoid_fat,
				    start, dest, pole_est);
  }

  if(pole_in_path){
    ROS_INFO("POLE IN PATH!!! AVOIDANCE ROUTINE ACTIVATED");
    avoidance_active = true;
  }
  
  if(avoidance_active && avoidance_counter == 0){
    ROS_INFO("UPDATING AVOID ENTRANCE BASED ON NEW POLE DATA...");
    dest.x = avoid_enter.x;
    dest.y = avoid_enter.y;
    forward = avoid_enter.forward;
  }
  else{
    ROS_INFO("POLE HAS MOVED OUT OF PATH FOR SOME REASON, RETURNING TO NORMAL DESTINATION...");
    dest.x = waypoints[0].x;
    dest.y = waypoints[0].y;
    dest.forward = waypoints[0].forward;
  }

}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  //grab the current ekf readings

  cur_pos = *pose_msg;
  
  ROS_INFO("Forward or Turn %d",forward_or_turn);
  if(forward_or_turn){
    //do the ol pid dance
    if(ye_ol_pid()){
      usleep(1000000);

      // if avoidance routine in progress, check to see if we've just reached
      // avoid_enter. if so, add the other two avoidance points and continue as normal.
      if(avoidance_active){
	if(avoidance_counter == 0){
	  //	  waypoints.push_back(avoid_fat);
	  ROS_INFO("AVOIDANCE ROUTINE FOUND 'AVOID_ENTER'. ADDING THIN/FAT & EXIT...");
	  waypoints.push_front(avoid_thin);
	  waypoints.push_front(avoid_exit);
	  start.x = avoid_enter.x;
	  start.y = avoid_enter.y;
	}
	avoidance_counter++;
	ROS_INFO("AVOIDANCE ROUTINE HAS REACHED AN AVOIDANCE POINT! AVOID_COUNTER = %d",avoidance_counter);

	if(avoidance_counter == 3)//shut off avoidance once we've done our 3 points
	  ROS_INFO("AVOIDANCE ROUTINE COMPLETED, AVOIDANCE OFF. RESUMING NORMAL WAYPOINTS...");
	  avoidance_active = false;
	  avoidance_counter = 0;
	}
      }

      if(next_waypoint()){
	ROS_INFO("Start: (%f,%f) Dest: (%f,%f) Fwd: %d",start.x,start.y,dest.x,dest.y,forward);
      }else{
	ROS_ERROR("Failed to call service request_next_waypoints");
      }
      forward_or_turn = 0;
    
      //reinitialize all errors to zero
      previous_error = 0;
      sum_of_errors = 0;
      total_num_of_errors = 0;
      error = 0;
      linear_vel = FAST_SPEED;
    
  }else{//we turnin'
    if(turn_to_heading()){
      forward_or_turn = 1;
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

  //WAYPOINTS ~~~~~~~~~~~~~~~~~~~~~
  waypoint_number = 0;
  avoidance_active = false;
  avoidance_counter = 0;
  pole_est.x = 0;
  pole_est.y = 0;

  nh.param("waypoints_filename", waypoints_filename, std::string("waypoints.txt"));
  
  ROS_INFO("Waypoints file:%s", waypoints_filename.c_str());
  bool file_good = read_in_waypoints();
  
  if(!file_good){
    ROS_ERROR("Error in waypoints service.");
    return 1;
  }
  ROS_INFO("Number of waypoints:%d", (int)waypoints.size());
  
  for(int i = 0; i < waypoints.size(); i++){
    ROS_INFO("(%f, %f)\t%s", waypoints[i].x, waypoints[i].y, waypoints[i].forward?"fwd":"back");
  }

  //grab the inital waypoint
  if(next_waypoint()){
    ROS_INFO("Start: (%f,%f) Dest: (%f,%f) Fwd: %s",start.x,start.y,dest.x,dest.y,forward?"true":"false");
  }else{
    ROS_ERROR("Failed to call service request_next_waypoints");
  }
  forward_or_turn = 1;//
  
  //PID ~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  std::string pose_namespace,cmd_vel_namespace,pole_namespace;

  //read in pid parameters
  nh.param("FAST_SPEED", FAST_SPEED, 0.0);
  nh.param("SLOW_SPEED", SLOW_SPEED, 0.0);
  nh.param("KP", KP, 0.0);
  nh.param("KI", KI, 0.0);
  nh.param("KD", KD, 0.0);
  nh.param("KP_SLOW", KP_SLOW, 0.0);
  nh.param("KI_SLOW", KI_SLOW, 0.0);
  nh.param("KD_SLOW", KD_SLOW, 0.0);

  nh.param("pose",pose_namespace,std::string("/redblade_ekf/2d_pose"));
  nh.param("cmd_vel",cmd_vel_namespace,std::string("/cmd_vel"));
  ROS_INFO("FAST: %f\tSLOW: %f\tKP: %f\t", FAST_SPEED, SLOW_SPEED, KP);
  ROS_INFO("Pose Namespace %s",pose_namespace.c_str());

  //Subscribe to Pose topic
  ros::Subscriber pose_sub = n.subscribe(pose_namespace, 1, poseCallback);

  //Subscribe to pole topic
  nh.param("pole",pole_namespace,std::string("/lidar/pole"));
  ros::Subscriber pole_sub = n.subscribe(pole_namespace, 1, poleCallback);

  //Set up cmd_vel publisher
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_namespace, 10);
  
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
    ros::spinOnce();
  }
  
}
