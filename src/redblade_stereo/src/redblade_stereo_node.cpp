#include "redblade_stereo.h"

ros::Publisher pub;
ros::Publisher test_pub;
ros::Publisher line_pub;
ros::Publisher transformed_pub;
redblade_stereo* redStereo;
geometry_msgs::Point localPolePoint;
geometry_msgs::Pose2D currentPose;
bool hasPoints = false;
std_msgs::Header h; 

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > filteredBackground(new pcl::PointCloud<pcl::PointXYZ>());

void pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  currentPose = *pose_msg;
}

// callback signature, assuming your points are pcl::PointXYZ type:
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input){
  //ROS_INFO("Point Cloud 2 Callback");  
  //pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr filteredGround,filteredBackground,pole;
  sensor_msgs::PointCloud2 test,line,transformed;
  h = input->header;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    filteredGround(new pcl::PointCloud<pcl::PointXYZ>());
  
  sensor_msgs::PointCloud2 output;
  //ROS_INFO("Converting point clouds");
  pcl::fromROSMsg(*input,*cloud);

  //First transform the point cloud
  redStereo->transform(cloud);
  pcl::toROSMsg(*cloud,transformed);
  transformed.header = h;
  transformed_pub.publish(transformed);
  //Then filter out ground
  //ROS_INFO("Filtering background");
  redStereo->filterBackground(cloud,filteredGround);
  //Then filter out background
  //ROS_INFO("Filtering ground");
  redStereo->filterGround(filteredGround,filteredBackground);
  pcl::toROSMsg(*filteredBackground,test);
  test.header = h;
  test_pub.publish(test);
  hasPoints = true;
  //Finally obtain pole location
  //ROS_INFO("Locating Pole");
  // bool found_pole = redStereo->findPole(filteredBackground,pole);
  // if(found_pole){
  //   pcl::toROSMsg(*pole,line);
  //   line.header = input->header;
  //   line_pub.publish(line);
       
  //   //ROS_INFO("Condensing Pole into Pole");
  //   //Obtain the pole point in the Bumblebee reference frame
  //   redStereo->cloud2point(pole,localPolePoint);
  //   hasPole = true;
  //   //pub.publish(polePoint);   
  // }else{
  //   //ROS_INFO("No Pole Found");
  // }
}
void publish_loop(){
  // if(!hasPoints)
  //   return;
  // sensor_msgs::PointCloud2 line;
  // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
  //   pole(new pcl::PointCloud<pcl::PointXYZ>());
  // bool found_pole = redStereo->findPole(filteredBackground,pole);
  // if(found_pole){
  //   pcl::toROSMsg(*pole,line);
  //   line.header = h;
  //   line_pub.publish(line);
       
  //   //ROS_INFO("Condensing Pole into Pole");
  //   //Obtain the pole point in the Bumblebee reference frame
  //   redStereo->cloud2point(pole,localPolePoint);
    
  //   //pub.publish(polePoint);   
  
  //   /*Convert coordinates from robot's local coordinate frame 
  //     to local ENU coordinate frame*/
  //   geometry_msgs::Point enuPolePoint;
  //   double xc    = localPolePoint.x;
  //   double yc    = localPolePoint.y;
  //   double theta = currentPose.theta;
  //   double x0    = currentPose.x;
  //   double y0    = currentPose.y;

  //   enuPolePoint.x = xc*cos(theta)-yc*sin(theta)+x0;
  //   enuPolePoint.y = xc*sin(theta)+yc*cos(theta)+y0;
  //   enuPolePoint.z = 0;
  //   pub.publish(enuPolePoint);
    
  // }
  // hasPoints = false;

}
int main(int argc, char** argv){
  ros::init(argc, argv, "redblade_stereo");
  //ros::NodeHandle n; 
  ros::NodeHandle nh;
  std::string stereo_namespace,pole_namespace,ekf_namespace;
  int queue_size;
  double ground_height,viewing_radius,pole_width,camera_height,camera_length_offset;
  std::string topic;
  
  //See odometry_skid_steer.h for all constants
  nh.param("queue_size", queue_size, 1);
  nh.param("stereo_namespace", stereo_namespace, std::string("/stereo_camera/points2"));
  nh.param("ekf_namespace", ekf_namespace, std::string("/redblade_ekf/2d_pose"));
  nh.param("pole_namespace", pole_namespace, std::string("/stereo_camera/pole"));
  nh.param("ground_height", ground_height, 0.5);
  nh.param("viewing_radius", viewing_radius, 3.0);
  nh.param("pole_width", pole_width, 0.1);
  nh.param("camera_height", camera_height, 1.67);
  nh.param("camera_length_offset", camera_length_offset, 0.55);

  ROS_INFO("Stereo Namespace %s",stereo_namespace.c_str());
  ROS_INFO("Ground Height %f Viewing Radius %f",ground_height,viewing_radius);
  
  //redblade_stereo redStereo(radius,height,width);
  redStereo = new redblade_stereo(viewing_radius,ground_height,pole_width,
				  camera_height,camera_length_offset);
 
  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2> (stereo_namespace, queue_size, cloud_callback);
  ros::Subscriber pose_sub  = nh.subscribe<geometry_msgs::Pose2D> (ekf_namespace, queue_size, pose_callback);
  //ros::Publisher line_pub;
  
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
