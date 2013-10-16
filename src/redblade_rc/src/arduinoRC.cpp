#include "arduinoRC.hpp"
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>

using namespace std;


int fb=0;
int lr=0;
int left_=0;
int right_=0;
int old_fb=0;
int old_lr=0;

double speedInMps = 0;
double linresolution = 1;
double angresolution = 1;
double roboteqUnits = 0.01425;//1 roboteq unit in m/s (roughly)
double wheelbase = 0.4712;//distance between wheels in meters
//int speedlimit = 70; //limit the number of roboteq units we can drive at (70 = 1 m/s)


ros::Publisher arduinoRC_pub;

arduinoRC::arduinoRC() {
  
}

arduinoRC::~arduinoRC() {

}

void rcCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    fb = msg->x; //(int)arduinoRC->arduino->getPulse(FB_PIN);
     lr = msg->y; // (int)arduinoRC->arduino->getPulse(LR_PIN);
    //cout << "Pulse width usec: " << fb << " and: " << lr << endl;
    fb -= 1420;
    //fb /= 5;
    fb /= 3;
    lr -= 1420;
    lr /= -20;
    //lr /= -15;
    if (abs(fb) < 10){//new remote = 10, old remote = 6
    fb = 0;
    }else{
      if(fb < 0){
	fb += 10;
      }else{
	fb -= 10;
      }
    }
    if (abs(lr) < 1){//new remote = 1, old remote = 3
      lr = 0;
    }else{
      if(lr < 0){
	lr += 1;
      }else{
	lr -= 1;
      }
    }

    if(!fb && old_fb) {
      fb = old_fb;
      old_fb = 0;
    }
    if(!lr && old_lr) {
      lr = old_lr;
      old_lr = 0;
    }

    left_ = fb + lr;
    right_ = fb - lr; 

    // left_ = min((int)(left_),speedlimit);
    // right_ = min((int)(right_),speedlimit);

    //calculate our speed and cap it at 1.5
    speedInMps = ((right_+left_)/2)*roboteqUnits/linresolution;

    if(speedInMps < 0){
      speedInMps = max(-1.5,speedInMps);
    }else{
      speedInMps = min(1.5,speedInMps);
    }

    //PUBLISH LINEAR & ANGULAR(RAD/SEC) VELOCITIES, BOTH INTS
    geometry_msgs::Twist msg_out;
    msg_out.linear.x = -speedInMps;
    msg_out.linear.y = 0;
    msg_out.linear.z = 0;
    msg_out.angular.x = 0;
    msg_out.angular.y = 0;
    msg_out.angular.z = ((right_-left_)/wheelbase)*roboteqUnits/angresolution;

    arduinoRC_pub.publish(msg_out);

    ROS_INFO("\nraw: %d, %d\nlinear: %f\nangular: %f\n\n",fb ,lr ,msg_out.linear.x,msg_out.angular.z);
}

int main(int argc, char** argv){

  // Initialize ROS
  ros::init(argc,argv,"arduinoRC");
  ros::NodeHandle n;

  // Create a ROS subscriber
  ros::Subscriber arduinoRC_sub = n.subscribe ("/arduino_out", 100, rcCallback);

  // Create a ROS publisher
  arduinoRC_pub = n.advertise<geometry_msgs::Twist>("Arduino_RC",100);
 
  // Spin
  ros::spin ();
}
