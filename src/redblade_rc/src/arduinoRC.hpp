#ifndef ARDUINO_RC_H
#define ARDUINO_RC_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class arduinoRC {
public:
  arduinoRC();
  void rcCallback(const geometry_msgs::Vector3::ConstPtr& msg);
  virtual ~arduinoRC(void);
  //  void operator()();

  
private:

  
};

#endif
