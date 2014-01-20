
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include "snowplow_pid/request_next_waypoints.h"
#include <string>
#include <cmath>

//Parameters that will be read in at runtime
double FAST_SPEED, SLOW_SPEED, KP, KI, KD, KP_SLOW, KI_SLOW, KD_SLOW;

//global stuff
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist vel_targets;
geometry_msgs::Pose2D cur_pos;
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
bool forward_or_turn;//1 forward


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
  double desired_heading = atan2(dest.y-cur_pos.y,
				 dest.x-cur_pos.x);
  
  wrap_pi(desired_heading);
  if(!forward){
    desired_heading -= M_PI;
    wrap_pi(desired_heading);
  }
  
  //usleep(50000);
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
  if(fabs(error) < 0.15){//CHANGEDBOB
    ROS_INFO("Final Error: %lf fabs(error) %lf abs(error) %lf",error,fabs(error),abs(error));
    vel_targets.linear.x = 0;
    vel_targets.angular.z = 0;
    return true;
  }
  return false;
}
/*This method calculates the "cross track error", or the perpendicular distance from
a line that the robot is trying to follow. It takes as inputs the starting point, the destination point,
and the current point
                       
                       current point
                            |\
                            | \
			    |  |->CTE
start                       | /                           end   
 ^__________________________|/_____________________________^

*/
double calculate_cte(){
  double rot;
  geometry_msgs::Pose2D start_rot, cur_rot;

  //find out how many rads we need to rotate the field by
  rot = atan2(dest.y-start.y, dest.x-start.x);

  //rotate start point
  start_rot.x = start.x * cos(-rot) - start.y * sin(-rot);
  start_rot.y = start.x * sin(-rot) + start.y * cos(-rot);

  //rotate current point
  cur_rot.x = cur_pos.x * cos(-rot) - cur_pos.y * sin(-rot);
  cur_rot.y = cur_pos.x * sin(-rot) + cur_pos.y * cos(-rot);

  //calculate cross track error
  return (cur_rot.y - start_rot.y);
}


/*This function calculates when the robot has crossed a line perpendicular
to the two points it's traveling between
                                           |
start                                   end| <--when this line is crossed  
-^---------------------------------------^-| <--this function returns
                                           | <--a negative number
                                           |

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
  double desired_heading, kp_corr, ki_corr, kd_corr, pid, distance, cte, correction_vel;
  //double current_heading = current_imu.z;
  double current_heading = cur_pos.theta;

  //TODO: implement a check for stuck method here
  
  /*if this is the first time this method has been called, let's just send her in a straight line
    for a very short peiod of time*/
  if(vel_targets.linear.x == 0){
    vel_targets.linear.x = SLOW_SPEED * (forward?(1):(-1));// m/s
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
  
  //calculate extra error from cte
  cte = calculate_cte();

  //when cte error is positive, the robot is to the left of the current desired path,
  //so our correction should be negative, i.e. a negative angular velocity
  correction_vel = 10;
  correction_vel *= (M_PI/180);
  if(fabs(cte) > 0.05 && fabs(cte) < 1){
    if(cte > 0){
      error -= (((fabs(cte)-0.05)/0.95) * correction_vel);
    }else{
      error += (((fabs(cte)-0.05)/0.95) * correction_vel);
    }
  }else if(fabs(cte) > 1){
    //apply constant correction for very large cte
    if(cte > 0){
      error -= correction_vel;
    }else{
      error += correction_vel;
    }
  }
  
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
  ROS_INFO("Error: %f\tCTE: %f\tCurrent Heading: %f\t Desired Heading: %f",(error*(180/M_PI)),cte,(current_heading*(180/M_PI)),(desired_heading*(180/M_PI)));
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

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  //grab the current ekf readings
  cur_pos = *pose_msg;
  ROS_INFO("Forward or Turn %d",forward_or_turn);
  if(forward_or_turn){
    //do the ol pid dance
    if(ye_ol_pid()){
      usleep(500000);
      //TODO: use service to grab the next waypoint
      snowplow_pid::request_next_waypoints srv;
      if(waypoint_client.call(srv)){
	start = srv.response.start;
	dest = srv.response.dest;
	forward = srv.response.forward;
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
      //CHANGEDBOB
      if(distance_to_goal() < 2){
	linear_vel = SLOW_SPEED
      }else{
	linear_vel = FAST_SPEED;
      }
    }
  }else{//we turnin'
    if(turn_to_heading()){
      usleep(500000);//CHANGEDBOB
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
  
  std::string pose_namespace,cmd_vel_namespace;

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

  //set up service to grab waypoints
  waypoint_client = n.serviceClient<snowplow_pid::request_next_waypoints>("request_next_waypoints");

  //grab the inital waypoint
  snowplow_pid::request_next_waypoints srv;
  if(waypoint_client.call(srv)){
    start = srv.response.start;
    dest = srv.response.dest;
    forward = srv.response.forward;
    ROS_INFO("Start: (%f,%f) Dest: (%f,%f) Fwd: %d",start.x,start.y,dest.x,dest.y,forward);
  }else{
    ROS_ERROR("Failed to call service request_next_waypoints");
  }
  forward_or_turn = 1;//

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

  //CHANGEDBOB
  ros::AsyncSpinner spinner(1);
  spinner.start();    
  while(ros::ok()){ 
    publish_loop();    
    cmd_vel_rate.sleep();
  }
  spinner.stop();  
  
  
}
