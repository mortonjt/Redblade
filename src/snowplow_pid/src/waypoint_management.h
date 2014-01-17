#include <ros/ros.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <deque>

#define ROBOTWIDTH 1.12 //maximum width with plow. we can change this...
#define ROBOTLENGTH 1.905 //max length with plow. also apt to change.
#define PASSINGBUFFER 0.1
#define STOPPINGBUFFER 0.3

struct Waypoint{
  double x,y;
  bool forward;
};

std::string waypoints_filename;
std::deque<Waypoint> waypoints;
Waypoint start, dest;
bool forward;
int waypoint_number;

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

bool next_waypoint(){
  //check to make sure we have a valid waypoint to give, if not, loop back to beginning
  //TODO

  if(waypoints.size() > 0){
    //populate waypoints
    start.x = waypoints[0].x;
    start.y = waypoints[0].y;
    dest.x = waypoints[1].x;
    dest.y = waypoints[1].y;
    forward = waypoints[1].forward;

    // remove waypoint that we've reached. should probably remove this
    // from inside the reached destination method.
    waypoints.pop_front();

    ROS_INFO("Next waypoints request received");
    return true;
  }
  else{
    ROS_WARN("No more waypoints!");
    return false;
  }
  //waypoint_number++;
  
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

bool checkForPole(Waypoint& p1, Waypoint& p2, Waypoint& p3, Waypoint& p4,
		  Waypoint a, Waypoint b, Waypoint c){

  double radius_parallel = ROBOT_LENGTH/2 + STOPPINGBUFFER;
  double radius_perp = ROBOT_WIDTH/2 + PASSINGBUFFER;

  // VARIABLE DEFINITIONS
  // a : 1st point in line
  // b : 2nd point in line
  // c : approximate location of pole
  // p1 : entrance point 
  // p2 : exit point
  // p3 : side point (thin)
  // p4 : side point (fat)
  // *all points are in ENU

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

  if(ec_dist < radius_perp){

    // line intersects with circle
    // find distance from proj point to center of circle
    double dt = sqrt( radius_parallel*radius_parallel - ec_dist*ec_dist );
    
    Waypoint temp1, temp2;
    temp1.x = (t-dt)*Dirx + a.x; 
    temp1.y = (t-dt)*Diry + a.y; 
    temp2.x = (t+dt)*Dirx + a.x; 
    temp2.y = (t+dt)*Diry + a.y; 
    
    // check distances to make sure entrance/exit points labeled correctly
    double dist_temp1 = sqrt( (temp1.x-a.x)*(temp1.x-a.x) + (temp1.y-a.y)*(temp1.y-a.y) );
    double ac_dist = sqrt( (c.x-a.x)*(c.x-a.x) + (c.y-a.y)*(c.y-a.y) );
    if(dist_temp1 < ac_dist){
      p1.x = temp1.x;
      p1.y = temp1.y;
      p2.x = temp2.x;
      p2.y = temp2.y;
    }
    else{
      p1.x = temp2.x;
      p1.y = temp2.y;
      p2.x = temp1.x;
      p2.y = temp1.y;
    }

    // this is a magic simplification of what was just done to find the
    // first two intersection points but now we're using the projected point
    // and the center of the circle to get a perpendicular line to find the
    // side waypoints... and I just sort of simplified it down a bit. 

    double perp_line_dist = sqrt((c.x-e.x)*(c.x-e.x) + (c.y-e.y)*(c.y-e.y));
    double perp_Dirx = (c.x-e.x)/perp_line_dist; 
    double perp_Diry = (c.y-e.y)/perp_line_dist;
    
    Waypoint tempside1, tempside2;

    tempside1.x = (perp_line_dist-radius_perp) * perp_Dirx + e.x;
    tempside1.y = (perp_line_dist-radius_perp) * perp_Diry + e.y;
    tempside2.x = (perp_line_dist+radius_perp) * perp_Dirx + e.x;
    tempside2.y = (perp_line_dist+radius_perp) * perp_Diry + e.y;

    // assign side points based on which one is thin(p3) / fat(p4)

    double dist_tempside1 = sqrt( (tempside1.x-e.x)*(tempside1.x-e.x) + (tempside1.y-e.y)*(tempside1.y-e.y) );

    if(dist_tempside1 < radius_perp){
      p3.x = tempside1.x;
      p3.y = tempside1.y;
      p4.x = tempside2.x;
      p4.y = tempside2.y;
    }
    else {
      p3.x = tempside2.x;
      p3.y = tempside2.y;
      p4.x = tempside1.x;
      p4.y = tempside1.y;
    }

    //DECIDE TO SEND THIN OR FAT

    //return true because pole is in path
    return true;

  }
  else if( ec_dist == radius_perp){ 
    // line is tangent to circle. let's not do anything about it for now.
    // I'm just hoping this never happens.
    return false;
  }
  else {
    // line doesn't intersect circle, ain't nothin' doin'.
    return false;
  }

}

