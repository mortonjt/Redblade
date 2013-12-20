#include "twist_converter.h"

static double clicks_per_m = 15768.6;
static double wheel_base_width; //Width between the base wheels


ros::Publisher robo_front_pub;
ros::Publisher robo_back_pub;
geometry_msgs::Twist cmd;

geometry_msgs::Twist front_target;
geometry_msgs::Twist back_target;

twist_converter::twist_converter(double eff_wheel_base){
  //Assumption: The instantaneous centers of rotation are symmetrical (aka yL = yR)
  
  this->yL=-eff_wheel_base/2;
  this->yR=eff_wheel_base/2;
}

twist_converter::~twist_converter(){
  
}

void twist_converter::getVelocities(geometry_msgs::Twist& robot_twist,
				    geometry_msgs::Twist& front_twist,
				    geometry_msgs::Twist& back_twist){
  double vx = robot_twist.linear.x;
  double w  = robot_twist.angular.z;
  double Vl = vx+yL*w;
  double Vr = vx+yR*w;
}

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg){
  cmd = *msg;
}

void Arduino_RC_Callback(const geometry_msgs::Twist::ConstPtr& msg) {
  //for now, we're just going to push the overall linear and angular velocity through
  //to each of the motor controllers, we'll figure out what's actually happening later
  front_target.linear.x = msg->linear.x;
  front_target.linear.y = msg->linear.y;
  front_target.linear.z = msg->linear.z;
  front_target.angular.x = msg->angular.x;
  front_target.angular.y = msg->angular.y;
  front_target.angular.z = msg->angular.z;
  
  back_target.linear.x = -msg->linear.x;
  back_target.linear.y = msg->linear.y;
  back_target.linear.z = msg->linear.z;
  back_target.angular.x = msg->angular.x;
  back_target.angular.y = msg->angular.y;
  back_target.angular.z = msg->angular.z;
  
}


/*This method is called once every 25 ms and will publish a Twist message for both the front and back roboteqs.*/
void publish_loop(twist_converter& twistConv){
  twistConv.getVelocities(cmd,front_target,back_target);
  //publish target Twist messages
  robo_front_pub.publish(front_target);
  robo_back_pub.publish(back_target);
}  

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "twist_conv_node");
  ros::NodeHandle n; //in the global namespace
  ros::NodeHandle nh("~");//local namespace, used for params
  double eff_wheel_base; //Calculated from calibration
  
  n.param("effective_wheel_base_width", eff_wheel_base, 2.0);
  n.param("wheel_base_width", wheel_base_width, 2.0);
  
  twist_converter twistConv(eff_wheel_base);

  //Subscribe to cmd_vel topic
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1, cmd_velCallback);

  //set up two publishers, one to publish a Twist message for each roboteq
  robo_front_pub = n.advertise<geometry_msgs::Twist>("roboteq_front/cmd_vel", 10);
  robo_back_pub = n.advertise<geometry_msgs::Twist>("roboteq_back/cmd_vel", 10);

  //Set up rate for cmd_vel topic to be published at
  ros::Rate cmd_vel_rate(40);//Hz

  //set up wheel base width
  wheel_base_width = 0.473;

  //initialize Twist messages to zeros, might not be necessary, but YOLO
  front_target.linear.x = 0;
  front_target.linear.y = 0;
  front_target.linear.z = 0;
  front_target.angular.x = 0;
  front_target.angular.y = 0;
  front_target.angular.z = 0;

  back_target.linear.x = 0;
  back_target.linear.y = 0;
  back_target.linear.z = 0;
  back_target.angular.x = 0;
  back_target.angular.y = 0;
  back_target.angular.z = 0;
    
  //publish cmd_vel topic every 25 ms (40 hz)
  while(ros::ok()){
    publish_loop(twistConv);
    ros::spinOnce();
    //sleep for a bit to stay at 40 hz
    cmd_vel_rate.sleep();
    
  }

}
