#ifndef ARDUINO_NODE_H
#define ARDUINO_NODE_H

#include "arduino.hpp"
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class arduinoNode {
public:
  arduinoNode();
  void inCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  virtual ~arduinoNode(void);
  arduino* arduinoCall;
  //  void operator()();

private:


};

#endif
