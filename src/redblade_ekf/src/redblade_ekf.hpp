#ifndef REDBLADE_EKF
#define REDBLADE_EKF

//#include "Arduino.hpp"
#include <ros/ros.h>
#include <kalman/ekfilter.hpp>
//#include <tf/transform_broadcaster.h>

class redblade_ekf : public Kalman::EKFilter<double,1> {
public:
  redblade_ekf();
  ~redblade_ekf();
  //Arduino* arduino;
  //  void operator()();

  //tf::TransformBroadcaster odom_broadcaster;

protected:
  void makeA();
  void makeH();
  void makeV();
  void makeR();
  void makeW();
  void makeQ();
  void makeProcess();
  void makeMeasure();
  
private:
  double dt;
  double beta_linear;
  double beta_angular;
  double sigma_w1;
  double sigma_w2;
  double sigma_w3;
  double sigma_w4;
  double sigma_w5;
  double sigma_w6;
  double sigma_z1;
  double sigma_z2;
  double sigma_z3;
  double sigma_z4;
  double sigma_z5;
};

typedef redblade_ekf::Vector Vector;
typedef redblade_ekf::Matrix Matrix;

#endif
