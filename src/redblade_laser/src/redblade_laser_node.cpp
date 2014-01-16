#include "redblade_laser.h"

ros::Publisher pub;
ros::Publisher local_pub;
ros::Publisher filtered_pub;
ros::Publisher transformed_pub;

sensor_msgs::LaserScan currentScan;
geometry_msgs::Pose2D currentPose;
laser_geometry::LaserProjection projector_;
bool hasPoints;
bool hasPose;
bool verbose;

redblade_laser* redLazer;

void pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  currentPose = *pose_msg;
  hasPose = true;
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  currentScan = *scan_in;
  hasPoints = true;
  
  if(verbose){
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan_in, cloud); 
    local_pub.publish(cloud);}
}

void publish_loop(){
  if(hasPoints and hasPose){
    sensor_msgs::PointCloud2 cloud,transformed,backFiltered;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
      pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
      filtered(new pcl::PointCloud<pcl::PointXYZ>());
    projector_.projectLaser(currentScan, cloud); 
    pcl::fromROSMsg(cloud,*pcl_cloud);
    redLazer->transformLaser2ENU(currentPose,pcl_cloud);
    redLazer->filterBackground(pcl_cloud,filtered);
    if(verbose){
      pcl::toROSMsg(*pcl_cloud,transformed);
      transformed.header = cloud.header;
      transformed.header.frame_id = "enu";      
      transformed_pub.publish(transformed);}
    if(verbose){    
      pcl::toROSMsg(*filtered,backFiltered);
      backFiltered.header = cloud.header;
      backFiltered.header.frame_id = "enu";      
      filtered_pub.publish(backFiltered);}    
    redLazer->addScan(filtered);
    if(redLazer->saturated()){
      geometry_msgs::Point pt;
      geometry_msgs::PointStamped enuStamped;
      bool foundPole = redLazer->findPole(pt,0.1);//10 cm
      if(foundPole){
	enuStamped.header.stamp = ros::Time::now();
	enuStamped.header.frame_id = "enu";
	enuStamped.point = pt;
	pub.publish(enuStamped);
	hasPoints = false;
	hasPose = false;
      }
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "redblade_laser");
  ros::NodeHandle nh;
  ros::NodeHandle n("~"); 
  std::string laser_namespace,pole_namespace,ekf_namespace,transformed_namespace;
  hasPoints = false;
  hasPose = false;
  std::string surveyFile;
  int queue_size;
  double laser_length_offset;
  bool searchSnowField,tripleI;
  n.param("queue_size", queue_size, 5);
  n.param("laser_namespace", laser_namespace, std::string("/scan"));
  n.param("ekf_namespace", ekf_namespace, std::string("/redblade_ekf/2d_pose"));
  n.param("pole_namespace", pole_namespace, std::string("/lidar/pole"));
  n.param("survey_file", surveyFile, std::string("~"));
  n.param("laser_length_offset", laser_length_offset, 0.3075);//TODO: Need more accurate measurement
  n.param("tripleI", tripleI, true);
  n.param("search_snowfield", searchSnowField, true);
  n.param("verbose", verbose, false);
  ROS_INFO("Verbose %d",verbose);  
  ROS_INFO("Search snowfield: %d",searchSnowField);
  ROS_INFO("Triple I: %d",tripleI);

  redLazer = new redblade_laser(surveyFile,searchSnowField,tripleI,laser_length_offset,queue_size);
  
  ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan> (laser_namespace, queue_size, scanCallback);
  ros::Subscriber pose_sub  = nh.subscribe<geometry_msgs::Pose2D> (ekf_namespace, 1, pose_callback);
  pub = nh.advertise<geometry_msgs::PointStamped> (pole_namespace, 1);

  local_pub = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/local", 1);
  filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/filtered", 1);
  transformed_pub = nh.advertise<sensor_msgs::PointCloud2> ("/lidar/transformed", 1);
  
  ros::AsyncSpinner spinner(2);
  spinner.start();    
  while(ros::ok()){ 
    publish_loop();
  }
  spinner.stop();  
}
