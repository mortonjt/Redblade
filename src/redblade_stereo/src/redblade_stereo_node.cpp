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
  if(hasPoints and hasPose){
    //ROS_INFO("Converting point clouds");
    pcl::fromROSMsg(rosCloud,*cloud);
    redStereo->transform(cloud);
    if(verbose){
      pcl::toROSMsg(*cloud,transformed);
      transformed.header = h;
      transformed_pub.publish(transformed);}
    redStereo->filterGround(cloud,filteredGround);
    redStereo->filterBackground(filteredGround,filteredBackground);
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
      redStereo->cloud2point(pole,localPolePoint); //Obtain the pole point in the Bumblebee reference frame
      /*Convert coordinates from robot's local coordinate frame to local ENU coordinate frame*/
      geometry_msgs::Point enuPolePoint;
      double xc      = localPolePoint.x;
      double yc      = localPolePoint.y;
      double theta   = currentPose.theta;
      double x0      = currentPose.x;
      double y0      = currentPose.y;
      enuPolePoint.x = xc*cos(theta)-yc*sin(theta)+x0;
      enuPolePoint.y = xc*sin(theta)+yc*cos(theta)+y0;
      enuPolePoint.z = 0;
      pub.publish(enuPolePoint);    
    }
    hasPoints = false;
    hasPose = false;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "redblade_stereo");
  //ros::NodeHandle n; 
  ros::NodeHandle nh;
  std::string stereo_namespace,pole_namespace,ekf_namespace;
  int queue_size;
  double ground_height,viewing_radius,viewing_width,pole_width,camera_height,camera_length_offset;
  std::string topic;
  hasPoints = false; 
  hasPose = false;
 
  //See odometry_skid_steer.h for all constants
  nh.param("queue_size", queue_size, 1);
  nh.param("stereo_namespace", stereo_namespace, std::string("/stereo_camera/points2"));
  nh.param("ekf_namespace", ekf_namespace, std::string("/redblade_ekf/2d_pose"));
  nh.param("pole_namespace", pole_namespace, std::string("/stereo_camera/pole"));
  nh.param("ground_height", ground_height, 1.0);
  nh.param("viewing_radius", viewing_radius, 10.0);
  nh.param("viewing_width", viewing_width, 1.5);
  nh.param("pole_width", pole_width, 0.1);
  nh.param("camera_height", camera_height, 1.67);
  nh.param("camera_length_offset", camera_length_offset, 0.55);
  nh.param("verbose", verbose, false);

  ROS_INFO("Stereo Namespace %s",stereo_namespace.c_str());
  ROS_INFO("Ground Height %f Viewing Radius %f",ground_height,viewing_radius);
  
  //redblade_stereo redStereo(radius,height,width);
  redStereo = new redblade_stereo(viewing_radius,viewing_width,
				  ground_height,pole_width,
				  camera_height,camera_length_offset);
 
  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2> (stereo_namespace, queue_size, cloud_callback);
  ros::Subscriber pose_sub  = nh.subscribe<geometry_msgs::Pose2D> (ekf_namespace, queue_size, pose_callback);
  
  // create a templated publisher
  pub = nh.advertise<geometry_msgs::Point> (pole_namespace, queue_size);
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
