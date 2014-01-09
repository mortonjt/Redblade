#include "redblade_stereo.h"

ros::Publisher pub;
ros::Publisher test_pub;
ros::Publisher line_pub;
ros::Publisher transformed_pub;
redblade_stereo* redStereo;
geometry_msgs::Point localPolePoint;
geometry_msgs::Pose2D currentPose;
sensor_msgs::PointCloud2 rosCloud;
std_msgs::Header h; 

bool hasPoints;
bool hasPose;
bool verbose;

void pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  currentPose = *pose_msg;
  hasPose = true;
  //ROS_INFO("Got pose");
}

// callback signature, assuming your points are pcl::PointXYZ type:
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input){
  h = input->header;
  rosCloud=*input; 
  hasPoints = true;
  //ROS_INFO("Got cloud");
}

void publish_loop(){
  sensor_msgs::PointCloud2 test,line,transformed,output;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    filteredGround(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    filteredBackground(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    pole(new pcl::PointCloud<pcl::PointXYZ>());  
  geometry_msgs::Point enuPolePoint;
  if(hasPoints and hasPose){
    //ROS_INFO("Converting point clouds");
    pcl::fromROSMsg(rosCloud,*cloud);
    redStereo->transformStereo2ENU(currentPose,cloud);
    if(verbose){
      pcl::toROSMsg(*cloud,transformed);
      transformed.header = h;
      transformed_pub.publish(transformed);}
    redStereo->filterGround(cloud,filteredGround);
    redStereo->filterBackground(currentPose,filteredGround,filteredBackground);
    if(verbose){
      pcl::toROSMsg(*filteredBackground,test);
      test.header = h;
      test_pub.publish(test);}
    bool found_pole = redStereo->findPole(filteredBackground,pole);
    if(found_pole){
      if(verbose){
	pcl::toROSMsg(*pole,line);
	line.header = h;
	line_pub.publish(line);}
      redStereo->cloud2point(pole,enuPolePoint); //Obtain the pole point in the Bumblebee reference frame
      /*Convert coordinates from robot's local coordinate frame to local ENU coordinate frame*/
      //redStereo->transformRobot2ENU(currentPose,localPolePoint,enuPolePoint);
      geometry_msgs::PointStamped enuStamped;
      enuStamped.header.stamp = ros::Time::now();
      enuStamped.header.frame_id = "stereo_camera";
      enuStamped.point = enuPolePoint;
      pub.publish(enuStamped);    
    }
    hasPoints = false;
    hasPose = false;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "redblade_stereo");
  ros::NodeHandle nh;
  ros::NodeHandle n("~"); 
  std::string stereo_namespace,pole_namespace,ekf_namespace;
  int queue_size;
  double ground_height,viewing_radius,viewing_width,pole_width,camera_height,camera_length_offset;  

  std::string survey_file;
  hasPoints = false; 
  hasPose = false;
 
  //See odometry_skid_steer.h for all constants
  n.param("queue_size", queue_size, 1);
  n.param("stereo_namespace", stereo_namespace, std::string("/stereo_camera/points2"));
  n.param("ekf_namespace", ekf_namespace, std::string("/redblade_ekf/2d_pose"));
  n.param("pole_namespace", pole_namespace, std::string("/stereo_camera/pole"));
  n.param("ground_height", ground_height, 1.0);
  n.param("viewing_radius", viewing_radius, 10.0);
  n.param("survey_file", survey_file, std::string("~"));
  n.param("viewing_width", viewing_width, 1.5);
  n.param("pole_width", pole_width, 0.1);
  n.param("camera_height", camera_height, 1.67);
  n.param("camera_length_offset", camera_length_offset, 0.55);
  n.param("verbose", verbose, false);

  ROS_INFO("Stereo Namespace %s",stereo_namespace.c_str());
  ROS_INFO("Ground Height %f Viewing Radius %f",ground_height,viewing_radius);
  ROS_INFO("Survey File %s",survey_file.c_str());
  ROS_INFO("Verbose %d",verbose);
  
  //redblade_stereo redStereo(radius,height,width);
  redStereo = new redblade_stereo(survey_file,
				  ground_height,pole_width,
				  camera_height,camera_length_offset);
  
  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2> (stereo_namespace, queue_size, cloud_callback);
  ros::Subscriber pose_sub  = nh.subscribe<geometry_msgs::Pose2D> (ekf_namespace, queue_size, pose_callback);
  
  // create a templated publisher
  pub = nh.advertise<geometry_msgs::PointStamped> (pole_namespace, queue_size);
  line_pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo_camera/line", queue_size);
  test_pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo_camera/test", queue_size);
  transformed_pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo_camera/transformed", queue_size);
  ros::AsyncSpinner spinner(2);
  spinner.start();    
  while(ros::ok()){ 
    publish_loop();
  }
  spinner.stop();
}
