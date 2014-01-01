#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <redblade_ax2550/StampedEncoders.h>
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

#include <string>
#include <cmath>
#include <thread>

#include "ax2550/ax2550.h"

using namespace ax2550;
using std::string;

AX2550 *mc_front;
AX2550 *mc_back;
ros::Publisher encoder_pub_front;
ros::Publisher encoder_pub_back;

static double ENCODER_RESOLUTION = 200*4;
static double clicks_per_m = 15768.6;
double wheel_circumference = 0.0;
double wheel_base_length = 0.0;
double wheel_diameter = 0.0;
double encoder_poll_rate;
//std::string front_or_back;
//std::string odom_frame_id;
size_t error_count;
double target_speed_front = 0.0;
double target_direction_front = 0.0;
double target_speed_back = 0.0;
double target_direction_back = 0.0;

double rot_cov = 0.0;
double pos_cov = 0.0;

static double A_MAX = 120.0;
static double B_MAX = 120.0;

// Persistent variables
double prev_x = 0, prev_y = 0, prev_w = 0;
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

void cmd_vel_frontCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(mc_front == NULL || !mc_front->isConnected())
        return;
    // Convert mps to rpm
    double A = msg->linear.x;
    double B = msg->angular.z * (wheel_base_length/2.0);
   
    double A_rel = A / 0.01425;//truly a magical number, no idea where i got it
    double B_rel = B / 0.01425;
    // ROS_INFO("Arpm: %f, Arel: %f, Brpm: %f, Brel: %f", A_rpm, A_rel, B_rpm, B_rel);
   
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
    target_speed_front = A_rel;
    target_direction_front = B_rel;
}

void cmd_vel_backCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(mc_back == NULL || !mc_back->isConnected())
        return;
    // Convert mps to rpm
    double A = msg->linear.x;
    double B = msg->angular.z * (wheel_base_length/2.0);
   
    double A_rel = A / 0.01425;//truly a magical number, no idea where i got it
    double B_rel = B / 0.01425;
    // ROS_INFO("Arpm: %f, Arel: %f, Brpm: %f, Brel: %f", A_rpm, A_rel, B_rpm, B_rel);
   
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
    target_speed_back = A_rel;
    target_direction_back = B_rel;
}

void controlLoop_front() {
    // ROS_INFO("Relative move commands: %f %f", target_speed, target_direction);
    try {
    	mc_front->move(target_speed_front, target_direction_front);
    } catch(const std::exception &e) {
    	if (string(e.what()).find("did not receive") != string::npos
         || string(e.what()).find("failed to receive an echo") != string::npos) {
            ROS_WARN("Error commanding the front motors: %s", e.what());
    	} else {
            ROS_ERROR("Error commanding the front motors: %s", e.what());
            mc_front->disconnect();
    	}
    }
}

void controlLoop_back() {
    // ROS_INFO("Relative move commands: %f %f", target_speed, target_direction);
    try {
    	mc_back->move(target_speed_back, target_direction_back);
    } catch(const std::exception &e) {
    	if (string(e.what()).find("did not receive") != string::npos
         || string(e.what()).find("failed to receive an echo") != string::npos) {
            ROS_WARN("Error commanding the back motors: %s", e.what());
    	} else {
            ROS_ERROR("Error commanding the back motors: %s", e.what());
            mc_back->disconnect();
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

void queryEncoders(AX2550* mc, redblade_ax2550::StampedEncoders& encoder_msg) {
   
    //using time delta as an error flag for early exits. set to -1
    //in case mc is disconnected.
    encoder_msg.encoders.time_delta = -1;

    // Make sure we are connected
    if(!ros::ok() || mc == NULL || !mc->isConnected())
        return;
    
    long encoder1, encoder2;
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
    
    // Convert to mps for each wheel from delta encoder ticks
    /*double left_v = encoder1 * 2*M_PI / ENCODER_RESOLUTION;
    left_v /= delta_time;
    // left_v *= encoder_poll_rate;
    double right_v = -encoder2 * 2*M_PI / ENCODER_RESOLUTION;
    right_v /= delta_time;
    // right_v *= encoder_poll_rate;
    */

    //redblade_ax2550::StampedEncoders encoder_msg;
    // encoder_msg.header.stamp = now;
    // encoder_msg.header.frame_id = "base_link";
    // encoder_msg.encoders.time_delta = delta_time;

    if(mc->front_or_back == "front"){ //it's the front
      encoder_msg.encoders.left_wheel = encoder1;
      encoder_msg.encoders.right_wheel = -encoder2;
    }else{
      encoder_msg.encoders.left_wheel = -encoder1;
      encoder_msg.encoders.right_wheel = encoder2;
    }

    //message successfully received. set error flag = 0, no error
    encoder_msg.encoders.time_delta = 0;

    //encoder_pub.publish(encoder_msg);

    /*double v = 0.0;
    double w = 0.0;
    
    double r_L = wheel_diameter/2.0;
    double r_R = wheel_diameter/2.0;
    
    v += r_L/2.0 * left_v;
    v += r_R/2.0 * right_v;

    w += r_R/wheel_base_length * right_v;
    w -= r_L/wheel_base_length * left_v;

    
    // Update the states based on model and input
    prev_x += delta_time * v
                          * cos(prev_w + delta_time * (w/2.0));
    
    prev_y += delta_time * v
                          * sin(prev_w + delta_time * (w/2.0));
    prev_w += delta_time * w;
    prev_w = wrapToPi(prev_w);
    
    // ROS_INFO("%f", prev_w);
    
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(prev_w);
    
    // Populate the msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.pose.pose.position.x = prev_x;
    odom_msg.pose.pose.position.y = prev_y;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.pose.covariance[0] = pos_cov;
    odom_msg.pose.covariance[7] = pos_cov;
    odom_msg.pose.covariance[14] = 1e100;
    odom_msg.pose.covariance[21] = 1e100;
    odom_msg.pose.covariance[28] = 1e100;
    odom_msg.pose.covariance[35] = rot_cov;
    
    // odom_msg.twist.twist.linear.x = v/delta_time;
    odom_msg.twist.twist.linear.x = v;
    // odom_msg.twist.twist.angular.z = w/delta_time;
    odom_msg.twist.twist.angular.z = w;
    
    odom_pub.publish(odom_msg);
    */ 
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
    ros::init(argc, argv, "ax2550_awd_node");
    ros::NodeHandle n("~");//in the local roboteq namespace
    ros::NodeHandle nh;    //global
    prev_time = ros::Time::now();

    // Serial port parameters
    std::string port_front, port_back;
    n.param("serial_port_front", port_front, std::string("/dev/motor_controller"));
    n.param("serial_port_back", port_back, std::string("/dev/motor_controller"));

    std::string cmd_vel_front_namespace,cmd_vel_back_namespace;
    n.param("cmd_vel_front", cmd_vel_front_namespace, std::string("/front_cmd_vel"));
    n.param("cmd_vel_back", cmd_vel_back_namespace, std::string("/rear_cmd_vel"));
   
    // Setup Encoder polling, might not need this, we should take it out later probably
    n.param("encoder_poll_rate", encoder_poll_rate, 25.0);
    ros::Rate encoder_rate(encoder_poll_rate);
   
    // Encoder Publishers
    encoder_pub_front = nh.advertise<redblade_ax2550::StampedEncoders>("encoders_front", 5);
    encoder_pub_back = nh.advertise<redblade_ax2550::StampedEncoders>("encoders_back", 5);

    // Wheel diameter parameter
    n.param("wheel_diameter", wheel_diameter, 0.395);
   
    wheel_circumference = wheel_diameter * M_PI;
   
    // Wheel base length
    n.param("wheel_base_length", wheel_base_length, 0.473);

    // cmd_vel Subscriber
    ros::Subscriber sub_front = nh.subscribe(cmd_vel_front_namespace, 1, cmd_vel_frontCallback);
    ros::Subscriber sub_back = nh.subscribe(cmd_vel_back_namespace, 1, cmd_vel_backCallback);
   
    // Spinner (2 threads, 1 for each callback)
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while(ros::ok()) {
        ROS_INFO("AX2550 connecting to port %s", port_front.c_str());
        try {
            mc_front = new AX2550();
            mc_front->warn = warnMsgCallback;
            mc_front->info = infoMsgCallback;
            mc_front->debug = debugMsgCallback;
            mc_front->connect(port_front);
	    mc_front->front_or_back = "front";
        } catch(std::exception &e) {
            ROS_ERROR("Failed to connect to the AX2550 (front): %s", e.what());
            if (mc_front != NULL) {
            	mc_front->disconnect();
            }
        }
        ROS_INFO("AX2550 connecting to port %s", port_back.c_str());
        try {
            mc_back = new AX2550();
            mc_back->warn = warnMsgCallback;
            mc_back->info = infoMsgCallback;
            mc_back->debug = debugMsgCallback;
            mc_back->connect(port_back);
	    mc_back->front_or_back = "back";
        } catch(std::exception &e) {
            ROS_ERROR("Failed to connect to the AX2550 (back): %s", e.what());
            if (mc_back != NULL) {
            	mc_back->disconnect();
            }
        }
        int count = 0;
        while(mc_front != NULL && mc_front->isConnected() &&
	      mc_back  != NULL && mc_back->isConnected()  && ros::ok() ) {
            
	    redblade_ax2550::StampedEncoders encoder_msg_front;
	    redblade_ax2550::StampedEncoders encoder_msg_back;

	    //query each roboteq in its own thread to populate an enc msg
	    ros::Time now = ros::Time::now();

	    std::thread query_front(queryEncoders,std::ref(mc_front),std::ref(encoder_msg_front));
	    std::thread query_back(queryEncoders,std::ref(mc_back),std::ref(encoder_msg_back));

	    query_front.join();
	    query_back.join();

	    double delta_time = (now - prev_time).toSec();
	    prev_time = now;

	    //publish encoder msgs
	    // time delta is being used as an error flag if queries exit early(wow, such good coding practices,amaze)
	    if(encoder_msg_front.encoders.time_delta == 0 && 
	       encoder_msg_back.encoders.time_delta == 0){
	          encoder_msg_front.header.stamp = now;
		  encoder_msg_front.encoders.time_delta = delta_time;
		  encoder_msg_back.header.stamp = now;
		  encoder_msg_back.encoders.time_delta = delta_time;

		  encoder_pub_front.publish(encoder_msg_front);
		  encoder_pub_back.publish(encoder_msg_front);
	    }

	    // enter control loop every other query
            if (count == 1) {
	        std::thread control_front(controlLoop_front);
		std::thread control_back(controlLoop_back);
		control_front.join();
		control_back.join();
                count = 0;
            } else {
                count += 1;
            }
	    
	    // encoder_rate.sleep();
        }
        if (mc_front != NULL || mc_back != NULL) {
        	delete mc_front;
		delete mc_back;
        }
        mc_front = NULL;
	mc_back = NULL;

        if(!ros::ok())
            break;
        ROS_INFO("Will try to reconnect to the AX2550 in 5 seconds.");
        for (int i = 0; i < 100; ++i) {
        	ros::Duration(5.0/100.0).sleep();
        	if (!ros::ok())
        		break;
        }
        target_speed_front = 0.0;
        target_direction_front = 0.0;
        target_speed_back = 0.0;
        target_direction_back = 0.0;
    }

    spinner.stop();
    
    return 0;
}
