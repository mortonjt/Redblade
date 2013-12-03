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
double overlap_width;

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

void save_waypoint_vector(){
  //save current waypoint vector to a file
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

void generate_single_i_waypoints(){
  
}

void generate_triple_i_waypoints(){

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

  //TODO: subscribe to any topics?

  //read in survey points
  read_in_survey_points();

  //generate waypoints
  if(single_i){
    generate_single_i_waypoints();
  }else{
    generate_triple_i_waypoints();
  }
}
