#include <ros/ros.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

/*
All ENU coordinates read in from the survey waypoints file are assumed to be in a coordinate frame defined as follows

___________________________________________________________
| |
| |
| |
| |
| |
| |
| |
| |
|__________________________________________________________|

^(0,0)
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

void save_waypoint_vector(std::string filename){
  //save current waypoint vector to a file
  std::ofstream file(filename.c_str());
  for(int i = 0; i < waypoints.size(); i++){
    file << waypoints[i][0] << "," << waypoints[i][1] << "," << waypoints[i][2] << "\n";
  }
  file.close();
}

void read_in_survey_points(){
  std::vector<std::string> lines;
  parse_file_to_vector(survey_file, lines);
  for(int i = 0; i < lines.size(); i++){
    std::vector<double> temp_doubles;
    split_to_double(lines[i], ',', temp_doubles);
    survey_points.push_back(temp_doubles);
  }
}

//TODO: howdo
double get_orientation(){
  double orientation = .75;//rads
}

/*
The lat, long will only be used to figure out the orientation of the field.
 */
void generate_single_i_waypoints(){
  //define a bunch of stuff needed for calculations
  std::vector<double> temp(3,0);
  field_length = 10.0;
  field_width = 1.0;
  
  //find orientation of field
  double field_angle = get_orientation();

  //TODO: changed start_of_snowfield here, might need to fix some stuff
  //generate ENU points in coordinate frame where field is facing E/W
  center_of_snowfield = 2.0;//meters from outer boundaries
  start_of_snowfield = 3.0;//wrt starting outer boundary
  end_of_snowfield = 2.0;//wrt ending outer boundary

  //i field is relatively simple, the number of points can be hardcoded
  
  //start point for robot
  temp[0] = start_of_snowfield - rotation_center_to_front - buffer;
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
  temp[1] = center_of_snowfield + (plow_width/2) - (overlap_width/2) + back_up_distance;
  temp[2] = 1;
  waypoints.push_back(temp);

  //now back up a little bit
  temp[1] = temp[1] - back_up_distance;
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

  //we'll use a 4 pass strategy, but we'll hit the outside first, then the inside, then the outside again
  
  //starting point
  temp[0] = start_of_snowfield - rotation_center_to_front - buffer;
  temp[1] = center_of_snowfield + (overlap_width/2) - plow_width + (overlap_width) - (plow_width/2);
  temp[2] = 1;

  //first point towards the end of the path
  temp[0] = start_of_snowfield + field_length + end_of_snowfield - rotation_center_to__front - buffer;
  temp[2] = 1;

  //back up a little bit
  temp[0] -= back_up_distance;
  temp[2] = 0;

  //turn 90 degrees counter clockwise and plow across
  
}

int main(int argc, char** argv){
  //Node setup
  ros::init(argc, argv, "path_planner_node");
  ros::NodeHandle n;//global namespace
  ros::NodeHandle nh("~");//local namespace

  //read in params from local namespace
  nh.param("survey_file", survey_file, std::string("survey_file.txt"));
  nh.param("i_waypoint_file", i_waypoint_file, std::string("i_waypoints.txt"));
  nh.param("triple_i_waypoint_file", triple_i_waypoint_file, std::string("triple_i_waypoints.txt"));
  nh.param("single_i", single_i, true);
  nh.param("overlap_width", overlap_width, 0.2);
  nh.param("plow_width", plow_width, 1.1);
  nh.param("rotation_center_to_front", rotation_center_to_front, 1.0);
  nh.param("rotation_center_to_rear", rotation_center_to_rear, 1.0);
  nh.param("buffer", buffer, 0.2);
  nh.param("back_up_distance", back_up_distance, 0.3);

  //TODO: subscribe to any topics?

  //read in survey points, in lat, long
  read_in_survey_points();

  //generate waypoints
  if(single_i){
    generate_single_i_waypoints();
    save_waypoint_vector(i_waypoint_file);
  }else{
    generate_triple_i_waypoints();
    save_waypoint_vector(triple_i_waypoint_file);
  }

}
