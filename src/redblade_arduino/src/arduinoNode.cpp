#include "arduinoNode.hpp"
#include "arduino.cpp"

#include <iostream>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include "ros/ros.h"
//#include "std_msgs/Int16MultiArray.h"
//#include "std_msgs/MultiArrayDimension.h"
//#include "std_msgs/MultiArrayLayout.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>

using namespace std;

arduinoNode* arduinoNodeCall;

// Constructor
arduinoNode::arduinoNode() {
  arduinoCall = new arduino("/dev/ttyACM0");
}

// Destructor
arduinoNode::~arduinoNode() {

}

void inCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  if(msg->x == 1) {
      arduinoNodeCall->arduinoCall->setDigital(6,1);
      std::cout << "Arduino Light ON\n";
  }
  else {
      arduinoNodeCall->arduinoCall->setDigital(6,0);
      std::cout << "Arduino Light OFF\n";
  }
}

int main(int argc, char** argv){
 float forward_backward_stick, left_right_stick;
 // unsigned int i,j,k;

  arduinoNodeCall = new arduinoNode();

  //int test = 1; //REMOVE

  ros::init(argc,argv,"arduino");
  ros::NodeHandle n;
  ros::Publisher arduino_pub = n.advertise<geometry_msgs::Vector3>("/arduino_out",100);
  ros::Subscriber arduino_sub = n.subscribe ("/arduino_in", 100, inCallback);
  ros::Rate loop_rate(20); //publish at 20 hz.

  std::cout << "Arduino Ready\n";
  usleep(250000);
  
  while(ros::ok()) {
    ros::spinOnce();

    forward_backward_stick = (float)arduinoNodeCall->arduinoCall->getPulse(FB_PIN);
    left_right_stick = (float)arduinoNodeCall->arduinoCall->getPulse(LR_PIN);

    //PUBLISH
    geometry_msgs::Vector3 msg;

    msg.x = forward_backward_stick;
    msg.y = left_right_stick;
    msg.z = 0.0;

    arduino_pub.publish(msg);
    
    loop_rate.sleep();
    
    // REMOVE START
    /*if(test==1){
      arduinoNodeCall->arduinoCall->setDigital(6,1);
      std::cout << "ON\n";
      test = 0;
    }
    else {
      arduinoNodeCall->arduinoCall->setDigital(6,0);
      std::cout << "OFF\n";
      test = 1;
    }

    usleep(500000);*/
    // REMOVE END

  }

  delete arduinoNodeCall;

  return 0;
}
