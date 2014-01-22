#include <ros/ros.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <deque>
//#include "common_deluxe_functions.h"
#include "boundaryCheck.h"

std::deque<Waypoint> waypoints;
Waypoint start, dest;
bool forward;
//int waypoint_number;

bool read_in_waypoints(std::string waypoints_filename){
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
		  Waypoint a, Waypoint b, Waypoint c, 
		  double field_orientation, bool is_single_i){

  avoidance_points.clear();
  bool approach_left, going_left;
  Waypoint entrancePoint,exitPoint,backupPoint;

  double heading = atan2((b.y-a.y),(b.x-a.x));

  double backup_dist = 1.5;
  double radius_front = ROBOTFRONT + 0.5;
  double radius_back = ROBOTBACK + 0.8;
  double radius_side = ROBOTWIDTH/2 + 0.5;

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
    
    entrancePoint.forward = 1;
    exitPoint.forward = 1;
    if(dist_temp1 < ac_dist){
      entrancePoint.x = temp1.x;
      entrancePoint.y = temp1.y;
    }
    else{
      entrancePoint.x = temp2.x;
      entrancePoint.y = temp2.y;
    }

    //make a circle of points to go to:

    // rotate points e and c to find if we're on the left
    // or right side of the pole
    rotation_matrix(e,-heading);
    rotation_matrix(c,-heading);

    if(c.y - e.y >= 0)
      approach_left = false;
    else
      approach_left = true;

    /* //UNCOMMENT TO MAKE THE ROBOT GO IN A CIRCLE: */

    /* double theta_increment = PI/4; */
    /* Waypoint nextPoint; */

    /* // in the coordinate frame of the robot heading  */
    /* for(int ii = -3; ii <= 7; ii++){ */
    /*   nextPoint.x = c.x + cos(ii*theta_increment)*radius_side; */
    /*   nextPoint.y = c.y + sin(ii*theta_increment)*radius_side; */
    /*   nextPoint.forward = 1; */

    /*   rotation_matrix(nextPoint,heading); */
    /*   avoidance_points.push_back(nextPoint); */
    /* } */

    Waypoint nextPoint,rightmost_point,leftmost_point, left_smooth_point,right_smooth_point;
    //remember, c and e are still in coord frame of robot heading

    //-135 deg point for approaching right side so it 
    //doesn't cut into the pole
    right_smooth_point.x = c.x + cos(-3/4*PI)*radius_side;
    right_smooth_point.y = c.y + sin(-3/4*PI)*radius_side;
    right_smooth_point.forward = 1;
    rotation_matrix(right_smooth_point,heading);

    //+135 deg point for approaching left side so it 
    //doesn't cut into the pole
    left_smooth_point.x = c.x + cos(3/4*PI)*radius_side;
    left_smooth_point.y = c.y + sin(3/4*PI)*radius_side;
    left_smooth_point.forward = 1;
    rotation_matrix(left_smooth_point,heading);

    //try -90 deg point (right side point)
    rightmost_point.x = c.x;
    rightmost_point.y = c.y - radius_side;
    rightmost_point.forward = 1;
    rotation_matrix(rightmost_point,heading);
      
    //next put in +90 deg point (left side point)
    leftmost_point.x = c.x;
    leftmost_point.y = c.y + radius_side;
    leftmost_point.forward = 1;
    rotation_matrix(leftmost_point,heading);

    if(!checkBoundaries(rightmost_point,is_single_i,field_orientation)){
      ROS_INFO("PLOWING AROUND THE RIGHT SIDE WOULD BE OUT OF BOUNDS (%f,%f), TRYING LEFT",
	       rightmost_point.x,rightmost_point.y);

      if(!checkBoundaries(leftmost_point,is_single_i,field_orientation)){
	ROS_INFO("PLOWING AROUND THE LEFT SIDE WOULD BE OUT OF BOUNDS (%f,%f), TIME FOR A HIT AND RUN.",
	       leftmost_point.x,leftmost_point.y);
	return false; //tell the pid that the pole has gone away.
      }
      going_left = true;
    }
    else{
      going_left = false;
    }

    if(approach_left && going_left){
      avoidance_points.push_back(leftmost_point);
      ROS_INFO("APPROACHING LEFT, GOING LEFT");
    }
    else if(!approach_left && going_left){
      //avoidance_points.push_back(left_smooth_point);
      avoidance_points.push_back(leftmost_point);
      ROS_INFO("APPROACHING RIGHT, GOING LEFT");
    }
    else if(approach_left && !going_left){
      //avoidance_points.push_back(right_smooth_point);
      avoidance_points.push_back(rightmost_point);
      ROS_INFO("APPROACHING LEFT, GOING RIGHT");
    }
    else if(!approach_left && !going_left){
      avoidance_points.push_back(rightmost_point);
      ROS_INFO("APPROACHING RIGHT, GOING RIGHT");
    }

    //next put in exit point (enough room for back end of robot to
    //swing in when it turns straight back to its orig heading)

    rotation_matrix(entrancePoint,-heading);

    exitPoint.x = c.x + radius_back;
    exitPoint.y = entrancePoint.y;
    exitPoint.forward = 1;
    rotation_matrix(exitPoint,heading);
    avoidance_points.push_back(exitPoint);

    //backup point
    backupPoint.x = entrancePoint.x - backup_dist;
    backupPoint.y = entrancePoint.y;
    backupPoint.forward = 0;
    rotation_matrix(backupPoint,heading);
    avoidance_points.push_front(backupPoint);

    rotation_matrix(entrancePoint,heading);
    avoidance_points.push_front(entrancePoint);

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

