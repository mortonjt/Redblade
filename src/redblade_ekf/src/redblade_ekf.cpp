#include <iostream>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <kalman/ekfilter.hpp>
#include "redblade_ekf.hpp"
#include <cstdlib>
#include <cmath>

using namespace Kalman;
using namespace std;


//Constructor
redblade_ekf::redblade_ekf() {
  //arduino = new Arduino("/dev/ttyACM0");
  setDim(6, 0, 6, 5, 5);//6 states, 0 inputs, 6 process noise RV, 5 measurements, 5 measurement noise RV
  dt = 0.2;//i have no idea what this needs to be
  beta_linear = 0.1;
  beta_angular = 0.1;
  sigma_w1 = sqrt(0.0025);//was .000025
  sigma_w2 = sqrt(0.0025);//was .000025
  sigma_w3 = sqrt(0.0001);//was 0.0001
  sigma_w4 = sqrt(0.1);
  sigma_w5 = sqrt(0.0625);//was.0625
  sigma_w6 = sqrt(0.0001156);
  
  sigma_z1 = sqrt(0.0025);
  sigma_z2 = sqrt(0.0025);
  sigma_z3 = sqrt(0.0025);
  sigma_z4 = sqrt(0.0000076);
  sigma_z5 = sqrt(0.0025);
}

redblade_ekf::~redblade_ekf(){
  //DESTRUCT IT
}

void redblade_ekf::makeProcess(){
  Vector x_(x.size());
  x_(1) = x(1) + (cos(x(3))*x(4)*dt);
  x_(2) = x(2) + (sin(x(3))*x(4)*dt);
  x_(3) = x(3) + (x(5)*dt);
  x_(4) = x(4) * exp(-beta_linear * dt);
  x_(5) = x(5) * exp(-beta_angular * dt);
  x_(6) = x(6);
  x.swap(x_);
}

void redblade_ekf::makeMeasure(){
  z(1) = x(1) + (cos(x(3))*(0.21));//0.21 is the distance from center of rotation to gps
  z(2) = x(2) + (sin(x(3))*(0.21)); 
  z(3) = x(4);
  z(4) = x(3) + x(6);
  z(5) = x(5);
}

//ouch
void redblade_ekf::makeA(){
  A(1,1) = 1.0;
  A(1,2) = 0.0;
  A(1,3) = -sin(x(3))*x(4)*dt;
  A(1,4) = cos(x(3))*dt;
  A(1,5) = 0.0;
  A(1,6) = 0.0;

  A(2,1) = 0.0;
  A(2,2) = 1.0;
  A(2,3) = cos(x(3))*x(4)*dt;
  A(2,4) = sin(x(3))*dt;
  A(2,5) = 0.0;
  A(2,6) = 0.0;
  
  A(3,1) = 0.0;
  A(3,2) = 0.0;
  A(3,3) = 1.0;
  A(3,4) = 0.0;
  A(3,5) = dt;
  A(3,6) = 0.0;

  A(4,1) = 0.0;
  A(4,2) = 0.0;
  A(4,3) = 0.0;
  A(4,4) = exp(-beta_linear*dt);
  A(4,5) = 0.0;
  A(4,6) = 0.0;

  A(5,1) = 0.0;
  A(5,2) = 0.0;
  A(5,3) = 0.0;
  A(5,4) = 0.0;
  A(5,5) = exp(-beta_angular*dt);
  A(5,6) = 0.0;
  
  A(6,1) = 0.0;
  A(6,2) = 0.0;
  A(6,3) = 0.0;
  A(6,4) = 0.0;
  A(6,5) = 0.0;
  A(6,6) = 1.0;
}

void redblade_ekf::makeW(){
  W(1,1) = 1.0;
  W(1,2) = 0.0;
  W(1,3) = 0.0;
  W(1,4) = 0.0;
  W(1,5) = 0.0;
  W(1,6) = 0.0;

  W(2,1) = 0.0;
  W(2,2) = 1.0;
  W(2,3) = 0.0;
  W(2,4) = 0.0;
  W(2,5) = 0.0;
  W(2,6) = 0.0;

  W(3,1) = 0.0;
  W(3,2) = 0.0;
  W(3,3) = 1.0;
  W(3,4) = 0.0;
  W(3,5) = 0.0;
  W(3,6) = 0.0;

  W(4,1) = 0.0;
  W(4,2) = 0.0;
  W(4,3) = 0.0;
  W(4,4) = 1.0;
  W(4,5) = 0.0;
  W(4,6) = 0.0;
  
  W(5,1) = 0.0;
  W(5,2) = 0.0;
  W(5,3) = 0.0;
  W(5,4) = 0.0;
  W(5,5) = 1.0;
  W(5,6) = 0.0;
  
  W(6,1) = 0.0;
  W(6,2) = 0.0;
  W(6,3) = 0.0;
  W(6,4) = 0.0;
  W(6,5) = 0.0;
  W(6,6) = 1.0;
}

void redblade_ekf::makeQ(){
  Q(1,1) = sigma_w1*sigma_w1;
  Q(1,2) = 0.0;
  Q(1,3) = 0.0;
  Q(1,4) = 0.0;
  Q(1,5) = 0.0;
  Q(1,6) = 0.0;

  Q(2,1) = 0.0;
  Q(2,2) = sigma_w2*sigma_w2;
  Q(2,3) = 0.0;
  Q(2,4) = 0.0;
  Q(2,5) = 0.0;
  Q(2,6) = 0.0;

  Q(3,1) = 0.0;
  Q(3,2) = 0.0;
  Q(3,3) = sigma_w3*sigma_w3;
  Q(3,4) = 0.0;
  Q(3,5) = 0.0;
  Q(3,6) = 0.0;

  Q(4,1) = 0.0;
  Q(4,2) = 0.0;
  Q(4,3) = 0.0;
  Q(4,4) = sigma_w4*sigma_w4*(1-exp(-2*beta_linear*dt));
  Q(4,5) = 0.0;
  Q(4,6) = 0.0;
  
  Q(5,1) = 0.0;
  Q(5,2) = 0.0;
  Q(5,3) = 0.0;
  Q(5,4) = 0.0;
  Q(5,5) = sigma_w5*sigma_w5*(1-exp(-2*beta_angular*dt));;
  Q(5,6) = 0.0;
  
  Q(6,1) = 0.0;
  Q(6,2) = 0.0;
  Q(6,3) = 0.0;
  Q(6,4) = 0.0;
  Q(6,5) = 0.0;
  Q(6,6) = sigma_w6*sigma_w6;
}

void redblade_ekf::makeH(){
  H(1,1) = 1.0;
  H(1,2) = 0.0;
  H(1,3) = -sin(x(3))*(0.21);
  H(1,4) = 0.0;
  H(1,5) = 0.0;
  H(1,6) = 0.0;
  
  H(2,1) = 0.0;
  H(2,2) = 1.0;
  H(2,3) = cos(x(3))*(0.21);
  H(2,4) = 0.0;
  H(2,5) = 0.0;
  H(2,6) = 0.0;
  
  H(3,1) = 0.0;
  H(3,2) = 0.0;
  H(3,3) = 0.0;
  H(3,4) = 1.0;
  H(3,5) = 0.0;
  H(3,6) = 0.0;
  
  H(4,1) = 0.0;
  H(4,2) = 0.0;
  H(4,3) = 1.0;
  H(4,4) = 0.0;
  H(4,5) = 0.0;
  H(4,6) = 1.0;
  
  H(5,1) = 0.0;
  H(5,2) = 0.0;
  H(5,3) = 0.0;
  H(5,4) = 0.0;
  H(5,5) = 1.0;
  H(5,6) = 0.0;
  
}

void redblade_ekf::makeV(){
  V(1,1) = 1.0;
  V(1,2) = 0.0;
  V(1,3) = 0.0;
  V(1,4) = 0.0;
  V(1,5) = 0.0;
  
  V(2,1) = 0.0;
  V(2,2) = 1.0;
  V(2,3) = 0.0;
  V(2,4) = 0.0;
  V(2,5) = 0.0;
  
  V(3,1) = 0.0;
  V(3,2) = 0.0;
  V(3,3) = 1.0;
  V(3,4) = 0.0;
  V(3,5) = 0.0;

  V(4,1) = 0.0;
  V(4,2) = 0.0;
  V(4,3) = 0.0;
  V(4,4) = 1.0;
  V(4,5) = 0.0;

  V(5,1) = 0.0;
  V(5,2) = 0.0;
  V(5,3) = 0.0;
  V(5,4) = 0.0;
  V(5,5) = 1.0;
}

void redblade_ekf::makeR(){
  R(1,1) = sigma_z1*sigma_z1;
  R(1,2) = 0.0;
  R(1,3) = 0.0;
  R(1,4) = 0.0;
  R(1,5) = 0.0;
  
  R(2,1) = 0.0;
  R(2,2) = sigma_z2*sigma_z2;
  R(2,3) = 0.0;
  R(2,4) = 0.0;
  R(2,5) = 0.0;
  
  R(3,1) = 0.0;
  R(3,2) = 0.0;
  R(3,3) = sigma_z3*sigma_z3;
  R(3,4) = 0.0;
  R(3,5) = 0.0;

  R(4,1) = 0.0;
  R(4,2) = 0.0;
  R(4,3) = 0.0;
  R(4,4) = sigma_z4*sigma_z4;
  R(4,5) = 0.0;

  R(5,1) = 0.0;
  R(5,2) = 0.0;
  R(5,3) = 0.0;
  R(5,4) = 0.0;
  R(5,5) = sigma_z5*sigma_z5;
}




/*int main(int argc, char** argv){
  redblade_ekf ekf;
  const unsigned n = 6;//number of states
  const unsigned m = 5;//number of measures
  
  //initial covariance of estimate
  static const double _P0[] = {0.0025, 0.0, 0.0, 0.0, 0.0, 0.0,
			     0.0, 0.0025, 0.0, 0.0, 0.0, 0.0,
			     0.0, 0.0, pow(1.57,2), 0.0, 0.0, 0.0,
			     0.0, 0.0, 0.0, .25, 0.0, 0.0,
			     0.0, 0.0, 0.0, 0.0, 0.0625, 0.0,
			     0.0, 0.0, 0.0, 0.0, 0.0, pow(1.57,2)};
  Matrix P0(n,n,_P0);
  
  //initial estimate
  Vector x(n);
  x(1) = 0.0;
  x(2) = 0.0;
  x(3) = 0.0;
  x(4) = 0.0;
  x(5) = 0.0;
  x(6) = 0.0;
  //intialize ze filter
  ekf.init(x, P0);


  /*ros::init(argc,argv,"Arduino_RC");
  ros::NodeHandle n;
  ros::Publisher Arduino_RC_pub = n.advertise<geometry_msgs::Twist>("Arduino_RC",100);
  ros::Rate loop_rate(20); //publish at 20 hz.
  //ros::init(argc, argv, "redblade_ekf_node");
  ros::NodeHandle n;
  ros::Publisher redblade_ekf_pub_ = n.advertise<geometry_msgs::Pose2D>("redblade_ekf",5);
  ros::Rate loop_rate(5);//5 Hz

  
  while(ros::ok()) {
    //PUBLISH LINEAR & ANGULAR(RAD/SEC) VELOCITIES, BOTH INTS
    /*geometry_msgs::Twist msg;
    msg.linear.x = speedInMps;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = ((right-left)/wheelbase)*roboteqUnits/angresolution;

    Arduino_RC_pub.publish(msg);
    
    loop_rate.sleep();
    
    
  }
  
  //delete ekf;
  
  return 0;
}
*/
// void main(int argc, char** argv){

//   arduino = new redblade_ekf();

//   ros::init(argc,argv,"Arduino_RC");
//   ros::NodeHandle n;
  
//   redblade_ekf::redblade_ekf()

// }
