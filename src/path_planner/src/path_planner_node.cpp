#include <ros/ros.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

/*
All ENU coordinates read in from the survey waypoints file are assumed to be in a coordinate frame defined as follows
TODO
*/


//parameters
std::string survey_file, i_waypoint_file, triple_i_waypoint_file;
bool single_i;
std::vector<std::vector<double> > survey_points;
std::vector<std::vector<double> > waypoints;
double overlap_width, plow_width, rotation_center_to_front, rotation_center_to_rear, buffer;
double field_length, field_width, back_up_distance;
double center_of_snowfield, start_of_snowfield, end_of_snowfield;


//method to split string by a delimiter
void split_to_double(const std::string &s, char delim, std::vector<double> &elements){
  std::stringstream ss(s);
  std::string item;
  while(std::getline(ss,item,delim)){
    elements.push_back(atof(item.c_str()));
  }
}

//convienence method so i don't have to do this over and over
void parse_file_to_vector(std::string &filename, std::vector<std::string> &lines){
  std::string item;
  std::ifstream file(filename.c_str());
  while(std::getline(file, item, '\n')){
    lines.push_back(item);
  }
}
//save the current waypoint vector to a csv file
void save_waypoint_vector(std::string filename){
  //save current waypoint vector to a file
  std::ofstream file(filename.c_str());
  for(int i = 0; i < waypoints.size(); i++){
    file << waypoints[i][0] << "," << waypoints[i][1] << "," << waypoints[i][2] << "\n";
  }
  file.close();
}

//read in the survey points
void read_in_survey_points(){
  std::vector<std::string> lines;
  parse_file_to_vector(survey_file, lines);
  for(int i = 0; i < lines.size(); i++){
    std::vector<double> temp_doubles;
    split_to_double(lines[i], ',', temp_doubles);
    survey_points.push_back(temp_doubles);
  }
}

//use two survey points to find the angle of the field with respect
//to an ENU coordinate frame
double get_orientation(){
  double orientation = atan2(survey_points[1][1]-survey_points[0][1],
			     survey_points[1][0]-survey_points[0][0]);
  return orientation;
}

/*
  generate the set of waypoints what would be used with a field
aligned facing at exactly 0 degrees
 */
void generate_single_i_waypoints(){
  //define a bunch of stuff needed for calculations
  std::vector<double> temp(3,0);
  field_length = 10.0;
  field_width = 1.0;
  
  //generate ENU points in coordinate frame where field is facing E/W
  center_of_snowfield = 2.0;//meters from outer boundaries
  start_of_snowfield = 3.0;//wrt starting outer boundary
  end_of_snowfield = 2.0;//wrt ending outer boundary

  //i field is relatively simple, the number of points can be hardcoded
  
  //start point for robot
  //temp[0] = start_of_snowfield - rotation_center_to_front - buffer;
  temp[0] = start_of_snowfield - rotation_center_to_front;
  temp[1] = center_of_snowfield - (plow_width/2) + (overlap_width/2);
  temp[2] = 1;//this one doesn't matter too much, shouldn't ever be used
  waypoints.push_back(temp);

  //first waypoint at the end of the field
  temp[0] = start_of_snowfield + field_length + end_of_snowfield - rotation_center_to_front - buffer;
  temp[2] = 1;//go forward to this point
  waypoints.push_back(temp);

  //now back up a little bit
  temp[0] = temp[0] - back_up_distance;
  temp[2] = 0;//go backwards to this point
  waypoints.push_back(temp);

  //now the robot will turn counter clockwise and go forward again
  temp[1] = center_of_snowfield + (plow_width/2) - (overlap_width/2) + (back_up_distance-0.2);//CHANGEDBOBTUES
  temp[2] = 1;
  waypoints.push_back(temp);

  //now back up a little bit
  temp[0] = start_of_snowfield + field_length + end_of_snowfield - rotation_center_to_rear - buffer - .25;//CHANGEDBOBTUES
  temp[1] = temp[1] - (back_up_distance-.2);//CHANGEDBOBTUES
  temp[2] = 0;
  waypoints.push_back(temp);

  //now the robot will turn counter clockwise and go forward again
  temp[0] = rotation_center_to_front + buffer;
  temp[2] = 1;
  waypoints.push_back(temp);

  //back up
  temp[0] = temp[0] + back_up_distance;
  temp[2] = 0;
  waypoints.push_back(temp);
}



/*
  generate the set of waypoints what would be used with a field
aligned facing at exactly 0 degrees
 */
void generate_triple_i_waypoints(){
  //define a bunch of stuff needed for calculations
  std::vector<double> temp(3,0);
  field_length = 10.0;
  field_width = 3.0;
  
  //find orientation of field
  double field_angle = get_orientation();

  //generate ENU points in coordinate frame where field is facing E/W
  center_of_snowfield = 3.5;//meters from outer boundaries
  start_of_snowfield = 3.0;//wrt starting outer boundary
  end_of_snowfield = 2.0;//wrt ending outer boundary

  //we'll use a 4/6 pass strategy, we'll hit the outside first, then the inside, then the outside again

  //first waypoint, needed for triple i because we start rotated by 90 degrees
  //temp[0] = 2.53;
  temp[0] = 1.715;
  temp[1] = 4.385;
  temp[2] = 0;
  waypoints.push_back(temp);
  
  //starting point, first we do the loop around the outside
  //temp[0] = start_of_snowfield - rotation_center_to_front - buffer;
  temp[0] = start_of_snowfield - rotation_center_to_front;
  temp[1] = center_of_snowfield + (overlap_width/2) - plow_width + (overlap_width) - (plow_width/2);
  temp[2] = 1;
  waypoints.push_back(temp);

  //first point towards the end of the path
  temp[0] = start_of_snowfield + field_length + end_of_snowfield - rotation_center_to_front - buffer;
  temp[2] = 1;
  waypoints.push_back(temp);

  //back up a little bit
  temp[0] -= back_up_distance;
  temp[2] = 0;
  waypoints.push_back(temp);

  //turn 90 degrees counter clockwise and plow across
  temp[1] = center_of_snowfield*2 - rotation_center_to_front - buffer;
  temp[2] = 1;
  waypoints.push_back(temp);

  //back up a bit
  temp[0] = start_of_snowfield + field_length + end_of_snowfield - rotation_center_to_rear - buffer;
  temp[1] = center_of_snowfield - (overlap_width/2) + plow_width - overlap_width + (plow_width/2);
  temp[2] = 0;
  waypoints.push_back(temp);

  //turn counter clockwise and go back towards the garage
  temp[0] = rotation_center_to_front + buffer;
  temp[2] = 1;
  waypoints.push_back(temp);
  
  //back up a bit
  temp[0] += back_up_distance;
  temp[2] = 0;
  waypoints.push_back(temp);

  //turn counter clockwise and go towards the start waypoint for the inner loop
  temp[0] = start_of_snowfield - rotation_center_to_front - (0.2);
  temp[1] = center_of_snowfield + (overlap_width/2) - (plow_width/2);
  temp[2] = 1;
  waypoints.push_back(temp);
  
  //first point towards the end of the path
  temp[0] = start_of_snowfield + field_length + end_of_snowfield - rotation_center_to_front - buffer;
  temp[2] = 1;
  waypoints.push_back(temp);

  //back up a little bit
  temp[0] -= back_up_distance;
  temp[2] = 0;
  waypoints.push_back(temp);

  //turn 90 degrees counter clockwise and plow across
  temp[1] = center_of_snowfield*2 - rotation_center_to_front - buffer;
  temp[2] = 1;
  waypoints.push_back(temp);  

  //back up a bit
  temp[0] = start_of_snowfield + field_length + end_of_snowfield - rotation_center_to_rear - buffer - .3;//changed
  temp[1] = center_of_snowfield - (overlap_width/2) + (plow_width/2);
  temp[2] = 0;
  waypoints.push_back(temp);

  //turn 90 degrees counter clockwise and go back to the garage
  temp[0] = rotation_center_to_front + buffer;
  temp[2] = 1;
  waypoints.push_back(temp);
  
  //back up a bit
  temp[0] += back_up_distance;
  temp[2] = 0;
  waypoints.push_back(temp);

}

//rotate a specified point by a given angle using the
//origin as the rotation center
void rotation_matrix(std::vector<double> &point, double theta){
  double x = point[0];
  double y = point[1];
  point[0] = x*cos(theta) - y*sin(theta);
  point[1] = x*sin(theta) + y*cos(theta);
}

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "path_planner_node");
  ros::NodeHandle n;//global namespace
  ros::NodeHandle nh("~");//local namespace

  //read in params from local namespace
  nh.param("survey_file", survey_file, std::string("survey_file.csv"));
  nh.param("single_i_waypoint_file", i_waypoint_file, std::string("single_i_waypoints.csv"));
  nh.param("triple_i_waypoint_file", triple_i_waypoint_file, std::string("triple_i_waypoints.csv"));
  nh.param("overlap_width", overlap_width, 0.2);
  nh.param("plow_width", plow_width, 1.1);
  nh.param("rotation_center_to_front", rotation_center_to_front, 1.0);
  nh.param("rotation_center_to_rear", rotation_center_to_rear, 1.0);
  nh.param("buffer", buffer, 0.2);
  nh.param("back_up_distance", back_up_distance, 0.3);

  //TODO: subscribe to any topics?

  //read in survey points, in ENU
  read_in_survey_points();

  //generate waypoints
  //if(single_i){
  //generate_single_i_waypoints();
    /*}else{
    generate_triple_i_waypoints();
    }*/

  //find how our field sits with repsect to ENU
  double orientation = get_orientation();

  //generate single i waypoints();
  generate_single_i_waypoints();

  //rotate waypoints by angle of field
  for(int i = 0; i < waypoints.size(); i++){
    rotation_matrix(waypoints[i], orientation);
  }

  //save single i waypoints
  save_waypoint_vector(i_waypoint_file);

  waypoints.clear();

  //generate triple i waypoints
  generate_triple_i_waypoints();

  //rotate waypoints by angle of field
  for(int i = 0; i < waypoints.size(); i++){
    rotation_matrix(waypoints[i], orientation);
  }

  //save triple i waypoints
  save_waypoint_vector(triple_i_waypoint_file);

  /*if(single_i){
    save_waypoint_vector(i_waypoint_file);
  }else{
    save_waypoint_vector(triple_i_waypoint_file);
    }*/


}
