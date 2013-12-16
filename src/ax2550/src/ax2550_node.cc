#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "ax2550/StampedEncoders.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
//#include <tf.h>

#include <string>
#include <cmath>

#include "ax2550/ax2550.h"

using namespace ax2550;
using std::string;

AX2550 *mc;
ros::Publisher odom_pub;
ros::Publisher encoder_pub;
tf::TransformBroadcaster *odom_broadcaster;
static double ENCODER_RESOLUTION = 200*4;
static double clicks_per_m = 15768.6;
double wheel_circumference = 0.0;
double wheel_base_length = 0.0;
double wheel_diameter = 0.0;
double encoder_poll_rate = 0.0;
std::string odom_frame_id = "";
size_t error_count;
volatile double target_speed = 0.0;
volatile double target_direction = 0.0;
long previous_encoder1 = 0;
long previous_encoder2 = 0;

//imu stuff
geometry_msgs::Vector3 orientation;
geometry_msgs::Vector3 prev_orientation;

double rot_cov = 0.0;
double pos_cov = 0.0;

//max speeds (in Roboteq units)
static double A_MAX = 120.0;
static double B_MAX = 120.0;

// Persistent variables
double x_pos = 0, y_pos = 0, theta = 0;
ros::Time prev_time;

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

void imu_data_callback(const geometry_msgs::Vector3::ConstPtr& msg){
  orientation.x = msg->x;
  orientation.y = msg->y;
  orientation.z = msg->z;
} 

void base_controller_callback(const geometry_msgs::Twist::ConstPtr& msg){
      if(mc == NULL || !mc->isConnected())
        return;
    // Convert mps to rpm
    double A = -msg->linear.x;
    double B = -msg->angular.z * (wheel_base_length/2.0);
    
    double A_rel = A / 0.01425;
    double B_rel = B / 0.01425;

    // Bounds check
    if(A_rel > A_MAX)
        A_rel = A_MAX;
    if(A_rel < -1*A_MAX)
        A_rel = -1*A_MAX;
    if(B_rel > B_MAX)
        B_rel = B_MAX;
    if(B_rel < -1*B_MAX)
        B_rel = -1*B_MAX;

    // Set the targets
    target_speed = A_rel;
    target_direction = B_rel;
}

void Arduino_RC_Callback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(mc == NULL || !mc->isConnected())
        return;
    // Convert mps to rpm
    double A = msg->linear.x;
    double B = msg->angular.z * (wheel_base_length/2.0);
    
    //double A_rpm = A * (60.0 / (M_PI*wheel_diameter));
    //double B_rpm = B * (60.0 / (M_PI*wheel_diameter));
    
    // Convert rpm to relative
    //double A_rel = (A_rpm * 200 * 64) / 58593.75;
    //double B_rel = (B_rpm * 200 * 64) / 58593.75;
    
    // ROS_INFO("Arpm: %f, Arel: %f, Brpm: %f, Brel: %f", A_rpm, A_rel, B_rpm, B_rel);
    
    double A_rel = A / 0.01425;
    double B_rel = B / 0.01425;

    // Bounds check
    if(A_rel > A_MAX)
        A_rel = A_MAX;
    if(A_rel < -1*A_MAX)
        A_rel = -1*A_MAX;
    if(B_rel > B_MAX)
        B_rel = B_MAX;
    if(B_rel < -1*B_MAX)
        B_rel = -1*B_MAX;

    // Set the targets
    target_speed = A_rel;
    target_direction = B_rel;
}

// void simple_goal_callback(const geometry_msgs::Twist::ConstPtr& msg) {
//     if(mc == NULL || !mc->isConnected())
//         return;
//     // Convert mps to rpm (necessary???)
//     double A = msg->linear.x;
//     double B = msg->angular.z * (wheel_base_length/2.0);
        
//     double A_rel = A / 0.01425;
//     double B_rel = B / 0.01425;

//     // Bounds check
//     if(A_rel > A_MAX)
//         A_rel = A_MAX;
//     if(A_rel < -1*A_MAX)
//         A_rel = -1*A_MAX;
//     if(B_rel > B_MAX)
//         B_rel = B_MAX;
//     if(B_rel < -1*B_MAX)
//         B_rel = -1*B_MAX;

//     // Set the targets
//     target_speed = A_rel;
//     target_direction = B_rel;
// }

void controlLoop() {
  //ROS_INFO("Relative move commands: %f %f", target_speed, target_direction);
    try {
    	mc->move(target_speed, target_direction);
    } catch(const std::exception &e) {
    	if (string(e.what()).find("did not receive") != string::npos
         || string(e.what()).find("failed to receive an echo") != string::npos) {
            ROS_WARN("Error commanding the motors: %s", e.what());
    	} else {
            ROS_ERROR("Error commanding the motors: %s", e.what());
            mc->disconnect();
    	}
    }
}

void errorMsgCallback(const std::string &msg) {
    ROS_ERROR("%s", msg.c_str());
}

void warnMsgCallback(const std::string &msg) {
    ROS_WARN("%s", msg.c_str());
}

void infoMsgCallback(const std::string &msg) {
    ROS_INFO("%s", msg.c_str());
}

void debugMsgCallback(const std::string &msg) {
    ROS_DEBUG("%s", msg.c_str());
}

void queryEncoders() {
    // Make sure we are connected
    if(!ros::ok() || mc == NULL || !mc->isConnected())
        return;
    
    long encoder1, encoder2;
    ros::Time now = ros::Time::now();
    // Retreive the data
    try {
        mc->queryEncoders(encoder1, encoder2, false);
        if (error_count > 0) {
            error_count -= 1;
        }
    } catch(std::exception &e) {
        if (string(e.what()).find("failed to receive ") != string::npos
         && error_count != 10) {
            error_count += 1;
            ROS_WARN("Error reading the Encoders: %s", e.what());    
        } else {
            ROS_ERROR("Error reading the Encoders: %s", e.what());
            mc->disconnect();
        }
        return;
    }
 
    //calculate change in time between this encoder poll and the previous
    double delta_time = (now - prev_time).toSec();
    //set prev_time to now so it can be used in next itertaion of loop
    prev_time = now;
    
    //Convert to m/s for each wheel from delta encoder ticks 157.686 clicks/cm
    //figure out relative changes in wheel encoders since absolute didn't work for our roboteq
    long change_in_encoder1 = -1 * (encoder1 - previous_encoder1);
    long change_in_encoder2 = encoder2 - previous_encoder2;
    //check to make sure these values aren't way too large (this happens if the kill switch is engaged then unengaged)
    if(abs(change_in_encoder1) > 15000 || abs(change_in_encoder2) > 15000){
      ROS_WARN("Change in encoder values too large, ignoring these\n\tEncoder 1: %ld\n\tEncoder2: %ld\n\tdt: %f\n", change_in_encoder1, change_in_encoder2, delta_time);
      change_in_encoder1 = 0;
      change_in_encoder2 = 0;
    }
    //convert changes in encoders to a distance traveled. 15768.6 clicks is equal to a distance of 1 meter
    double distance_traveled = ((change_in_encoder1 + change_in_encoder2) / 2) / clicks_per_m;
    //find the velocities of each wheel by dividing distance traveled by delta_time
    double left_v = (change_in_encoder1 / clicks_per_m) / delta_time;
    double right_v = (change_in_encoder2 / clicks_per_m) / delta_time;
    //compute a combined velocity for the entire platform
    //this will be the velocity that the base_link frame moves in the odom_frame
    double combined_v = (left_v + right_v)/2;
    //compute the change in theta with the imu z gyro
    double theta_delta = 0;
    if(change_in_encoder1 == 0 && change_in_encoder2 == 0){
      theta_delta = 0;
    }else{
      theta_delta = (orientation.z - prev_orientation.z);
    }
    //compute the velocity of theta
    double theta_v = theta_delta / delta_time;
    
    
    
    //ROS_INFO("left: %f\nright: %f\n", left_v, right_v);
    
    // Convert to mps for each wheel from delta encoder ticks
    //double left_v = encoder1 * 2*M_PI / ENCODER_RESOLUTION;
    //left_v /= delta_time;
    // left_v *= encoder_poll_rate;
    //double right_v = -encoder2 * 2*M_PI / ENCODER_RESOLUTION;
    //right_v /= delta_time;
    // right_v *= encoder_poll_rate;
    
    //construct encoder message and publish
    ax2550::StampedEncoders encoder_msg;
    
    encoder_msg.header.stamp = now;
    encoder_msg.header.frame_id = "base_link";
    encoder_msg.encoders.time_delta = delta_time;
    encoder_msg.encoders.left_wheel = change_in_encoder1;
    encoder_msg.encoders.right_wheel = change_in_encoder2;
    
    encoder_pub.publish(encoder_msg);

    //rotationWithOdometry = (rad2deg)*(differenceOfDeltas/CLICKS_PER_CM)/WIDTH_OF_WHEELBASE;
    
    //rotation in radians
    //double rotation_with_odom = ((change_in_encoder1-change_in_encoder2) / 
    //clicks_per_m) / wheel_base_length;
    double delta_x = distance_traveled * cos(prev_orientation.z);
    double delta_y = distance_traveled * sin(prev_orientation.z);

    double velocity_x = delta_x / delta_time;
    double velocity_y = delta_y / delta_time;
    
    x_pos += delta_x;
    y_pos += delta_y;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(orientation.z);
    
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.195;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster->sendTransform(odom_trans);
 
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_id;
 
    //set the position
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
 
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = combined_v;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = theta_v;
 
    //TODO: covariance stuff
    odom.pose.covariance[0] = pos_cov;
    odom.pose.covariance[7] = pos_cov;
    odom.pose.covariance[14] = 1e100;
    odom.pose.covariance[21] = 1e100;
    odom.pose.covariance[28] = 1e100;
    odom.pose.covariance[35] = rot_cov;

    //publish the message
    odom_pub.publish(odom);
 
    //update stuff so we can compute changes on next loop iteration
    previous_encoder1 = encoder1;
    previous_encoder2 = encoder2;
    prev_orientation.x = orientation.x;
    prev_orientation.y = orientation.y;
    prev_orientation.z = orientation.z;
    
        
    // TODO: Add TF broadcaster
    // geometry_msgs::TransformStamped odom_trans;
    //     odom_trans.header.stamp = now;
    //     odom_trans.header.frame_id = "odom";
    //     odom_trans.child_frame_id = "base_footprint";
    // 
    //     odom_trans.transform.translation.x = prev_x;
    //     odom_trans.transform.translation.y = prev_y;
    //     odom_trans.transform.translation.z = 0.0;
    //     odom_trans.transform.rotation = quat;
    //     
    //     odom_broadcaster->sendTransform(odom_trans);
}

int main(int argc, char **argv) {
    // Node setup
    ros::init(argc, argv, "ax2550_node");
    ros::NodeHandle n;
    prev_time = ros::Time::now();
    
    // Serial port parameter
    std::string port;
    n.param("serial_port", port, std::string("/dev/ttyUSB1"));
    
    // Wheel diameter parameter
    n.param("wheel_diameter", wheel_diameter, 0.395);
    
    wheel_circumference = wheel_diameter * M_PI;
    
    // Wheel base length
    n.param("wheel_base_length", wheel_base_length, 0.473);
    
    // Odom Frame id parameter
    n.param("odom_frame_id", odom_frame_id, std::string("odom"));

    // Load up some covariances from parameters
    n.param("rotation_covariance",rot_cov, 1.0);
    n.param("position_covariance",pos_cov, 1.0);
    
    // Setup Encoder polling
    n.param("encoder_poll_rate", encoder_poll_rate, 10.0);
    ros::Rate encoder_rate(encoder_poll_rate);

    //initialize orientation vector
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;

    prev_orientation.x = 0.0;
    prev_orientation.y = 0.0;
    prev_orientation.z = 0.0;
    
    // Odometry Publisher
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);
    
    // Encoder Publisher
    encoder_pub = n.advertise<ax2550::StampedEncoders>("encoders", 5);
    
    // TF Broadcaster
    odom_broadcaster = new tf::TransformBroadcaster;
    
    // Arduino_RC Subscriber
    ros::Subscriber sub_arduino = n.subscribe("Arduino_RC", 5, Arduino_RC_Callback);
    
    //Base Controller Subscriber
    ros::Subscriber sub_base_controller = n.subscribe("cmd_vel", 5, base_controller_callback);

    // simple goal Subscriber
    //ros::Subscriber sub_goals = n.subscribe("simple_goals", 5, simple_goal_callback);

    //IMU subscriber
    ros::Subscriber sub_imu = n.subscribe("imu/integrated_gyros", 5, imu_data_callback);
    
    // Spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()) {
        ROS_INFO("AX2550 connecting to port %s", port.c_str());
        try {
            mc = new AX2550();
            mc->warn = warnMsgCallback;
            mc->info = infoMsgCallback;
            mc->debug = debugMsgCallback;
            mc->connect(port);
        } catch(std::exception &e) {
            ROS_ERROR("Failed to connect to the AX2550: %s", e.what());
            if (mc != NULL) {
            	mc->disconnect();
            }
        }
        int count = 0;
        while(mc != NULL && mc->isConnected() && ros::ok()) {
	  queryEncoders();
	  if (count == 1) {
	    controlLoop();
	    //ROS_INFO("Made it through control loop");
	    count = 0;
	  } else {
	    count += 1;
	  }
	  //TODO: BOB IS THIS NEXT LINE CORRECT!? I DON'T KNOW!!!!
	  //Fuck if i know breh
	  encoder_rate.sleep();
        }
        if (mc != NULL) {
	  delete mc;
        }
        mc = NULL;
        if(!ros::ok())
	  break;
        ROS_INFO("Will try to reconnect to the AX2550 in 5 seconds.");
        for (int i = 0; i < 100; ++i) {
	  ros::Duration(5.0/100.0).sleep();
	  if (!ros::ok())
	    break;
        }
        target_speed = 0.0;
        target_direction = 0.0;
    }

    spinner.stop();
    
    return 0;
}
