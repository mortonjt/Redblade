#include <ros/ros.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <deque>

#define ROBOTWIDTH 1.12 //maximum width with plow. we can change this...
#define ROBOTFRONT 1.285 //max length with plow. also apt to change.
#define ROBOTBACK 0.615
#define PI 3.141592653589
#define BUFFER 0.25

struct Waypoint{
  double x,y;
  bool forward;
};

std::string waypoints_filename;
std::deque<Waypoint> waypoints;
Waypoint start, dest;
bool forward;
int waypoint_number;

void rotation_matrix(Waypoint &point, double theta){
  double x = point.x;
  double y = point.y;
  point.x = x*cos(theta) - y*sin(theta);
  point.y = x*sin(theta) + y*cos(theta);
}

void split_to_double(const std::string &s, char delim, std::vector<double> &elements){
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss,item,delim)){
    elements.push_back(atof(item.c_str()));
  }
}

bool read_in_waypoints(){
  //first, read in each line
  std::vector<std::string> elements;
  std::string item;
  std::ifstream file(waypoints_filename.c_str());
  while(std::getline(file, item, '\n')){
    elements.push_back(item);
    ROS_INFO("Read item:%s", (char*)item.c_str());
  }

  //split each line
  for(int i = 0; i < elements.size(); i++){
    std::vector<double> delimited_line;
    split_to_double(elements[i], ',', delimited_line);
    Waypoint temp;
    temp.x = delimited_line[0];
    temp.y = delimited_line[1];
    temp.forward = delimited_line[2];
    waypoints.push_back(temp);
  }
  
  return true;
}

bool next_waypoint(bool avoidance_active){
  //check to make sure we have a valid waypoint to give, if not, loop back to beginning
  
  if(waypoints.size() > 0){
    //populate waypoints
    start.x = waypoints[0].x;
    start.y = waypoints[0].y;
    dest.x = waypoints[1].x;
    dest.y = waypoints[1].y;
    forward = waypoints[1].forward;

    //push top waypoint onto the back of the queue if not 
    //an avoidance waypoint
    if(avoidance_active){
      waypoints.pop_front();
    }
    else{
      waypoints.push_back(waypoints.front());
      waypoints.pop_front();
    }

    ROS_INFO("Next waypoints request received");
    return true;
  }
  else{
    //we should never get here because we recycle our points
    ROS_WARN("No more waypoints!");
    return false;
  }
  
}

//finds the nearest point from point to the line made by points a and b
double projectPoint(Waypoint p1, Waypoint a, Waypoint b){

  double line_dist = sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
  double Dirx = (b.x-a.x)/line_dist; //direction vector x
  double Diry = (b.y-a.y)/line_dist; //direction vector y
  double t = Dirx*(p1.x-a.x) + Diry*(p1.y-a.y);
  
  //now line eq. is x = Dirx*t + a.x, y = Diry*t + a.y , 0 <= t <= 1

  //find coordinates of projected point on line
  Waypoint projPoint;
  projPoint.x = t * Dirx + a.x;
  projPoint.y = t * Diry + a.y;
  
  //find distance from proj point to center of circle
  double point_line_dist = sqrt( (projPoint.x-p1.x)*(projPoint.x-p1.x) + (projPoint.y-p1.y)*(projPoint.y-p1.y) );

  return point_line_dist;
}

bool checkForPole(std::deque<Waypoint>& avoidance_points,
		  Waypoint a, Waypoint b, Waypoint c, double heading){

  avoidance_points.clear();
  bool is_left;
  Waypoint entrance;

  double radius_front = ROBOTFRONT + BUFFER;
  //double radius_back = ROBOTBACK + BUFFER;
  double radius_side = ROBOTWIDTH/2 + BUFFER;

  // VARIABLE DEFINITIONS
  // avoidance_points: list of avoidance points to return
  // a : 1st point in line
  // b : 2nd point in line
  // c : approximate location of pole

  // this could be replaced by a call to point_line_dist, 
  // but I need waypoint 'e' coords & Dirx/Diry and I don't really want to make
  // point_line_dist have to spit out those items for every call.
  double line_dist = sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
  double Dirx = (b.x-a.x)/line_dist; //direction vector x  
  double Diry = (b.y-a.y)/line_dist; //direction vector y
  double t = Dirx*(c.x-a.x) + Diry*(c.y-a.y);
  
  //now line eq. is x = Dirx*t + a.x, y = Diry*t + a.y , 0 <= t <= 1

  //find coordinates of projected point on line
  Waypoint e;
  e.x = t * Dirx + a.x;
  e.y = t * Diry + a.y;
  
  //find distance from proj point to center of circle
  double ec_dist = sqrt( (e.x-c.x)*(e.x-c.x) + (e.y-c.y)*(e.y-c.y) );

  // find out if the line even intersects with the circle
  if(ec_dist < radius_side){

    // line intersects with circle
    // find distance from proj point to center of circle
    double dt = sqrt( radius_front*radius_front - ec_dist*ec_dist );
    
    Waypoint temp1, temp2;
    temp1.x = (t-dt)*Dirx + a.x; 
    temp1.y = (t-dt)*Diry + a.y; 
    temp2.x = (t+dt)*Dirx + a.x; 
    temp2.y = (t+dt)*Diry + a.y; 
    
    // check distances to make sure entrance/exit points labeled correctly
    double dist_temp1 = sqrt( (temp1.x-a.x)*(temp1.x-a.x) + (temp1.y-a.y)*(temp1.y-a.y) );
    double ac_dist = sqrt( (c.x-a.x)*(c.x-a.x) + (c.y-a.y)*(c.y-a.y) );
    
    entrance.forward = 1;
    if(dist_temp1 < ac_dist){
      entrance.x = temp1.x;
      entrance.y = temp1.y;
    }
    else{
      entrance.x = temp2.x;
      entrance.y = temp2.y;
    }

    //make a circle of points to go to:

    // rotate points e and c to find if we're on the left
    // or right side of the pole
    rotation_matrix(e,-heading);
    rotation_matrix(c,-heading);

    if(c.y - e.y >= 0)
      is_left = false;
    else
      is_left = true;

    double theta_increment = PI/4;
    Waypoint nextPoint;

    // in the coordinate frame of the robot heading 
    for(int ii = -3; ii <= 7; ii++){
      nextPoint.x = c.x + cos(ii*theta_increment)*radius_side;
      nextPoint.y = c.y + sin(ii*theta_increment)*radius_side;
      nextPoint.forward = 1;

      rotation_matrix(nextPoint,heading);
      avoidance_points.push_back(nextPoint);
    }

    // remove the first point if we're approaching from the right
    // because it's only there for a smooth entrance from the left
    if(!is_left){
      avoidance_points.pop_front();
      ROS_INFO("WE'RE APPROACHING POLE FROM RIGHT SIDE");
    }

    avoidance_points.push_front(entrance);

    //return true because pole is in path
    return true;

  }
  else if( ec_dist == radius_side){ 
    // line is tangent to circle. let's not do anything about it for now.
    // I'm just hoping this never happens.
    return false;
  }
  else {
    // line doesn't intersect circle, ain't nothin' doin'.
    return false;
  }

}

