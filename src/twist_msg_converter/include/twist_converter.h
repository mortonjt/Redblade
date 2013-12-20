#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include <string>
#include <cmath>

/*

Orientation
 x
 ^   z
 |  /
 | /
 |/
 *-------> y
 
 Assumption: z=0 since we are on a fairly level ground
 */


//Some vector algebra methods
//Add two vectors
geometry_msgs::Vector3 add(geometry_msgs::Vector3 V1,geometry_msgs::Vector3 V2){
  geometry_msgs::Vector3 V;
  V.x = V1.x+V2.x;
  V.y = V1.y+V2.y;
  V.z = V1.z+V2.z;
  return V;};
//Cross product between two vectors
geometry_msgs::Vector3 xprod(geometry_msgs::Vector3 V1,geometry_msgs::Vector3 V2){
  geometry_msgs::Vector3 V;
  V.x = V1.y*V2.z-V1.z*V2.y;
  V.y = V1.x*V2.z-V1.z*V2.x;
  V.z = V1.x*V2.y-V1.y*V2.x;
  return V;};
double mag(geometry_msgs::Vector3 V){
  return sqrt(V.x*V.x+V.y*V.y+V.z*V.z);}
geometry_msgs::Vector3 createVector3(double x, double y, double z){
  geometry_msgs::Vector3 V;
  V.x = x;
  V.y = y;
  V.z = z;
  return V;};


class twist_converter{
 public:
  double yL, yR; //Instantaneous Center of Rotation for y coordinates
  
  twist_converter(double eff_wheel_base);
  
  ~twist_converter();
  
  void getVelocities(geometry_msgs::Twist& robot_twist,
		     geometry_msgs::Twist& front_twist,
		     geometry_msgs::Twist& back_twist);
  

};
