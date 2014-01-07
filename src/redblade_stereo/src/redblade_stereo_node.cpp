#include "redblade_stereo.h"

ros::Publisher pub;
ros::Publisher test_pub;
ros::Publisher line_pub;
redblade_stereo* redStereo;
geometry_msgs::Point localPolePoint;
bool hasPole = false;

void pose_callback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  if(hasPole){
    /*Convert coordinates from robot's local coordinate frame 
      to local ENU coordinate frame*/
    geometry_msgs::Point enuPolePoint;
    double xc    = localPolePoint.x;
    double yc    = localPolePoint.y;
    double theta = pose_msg->theta;
    double x0    = pose_msg->x;
    double y0    = pose_msg->y;

    enuPolePoint.x = xc*cos(theta)-yc*sin(theta)+x0;
    enuPolePoint.y = xc*sin(theta)+yc*cos(theta)+y0;
    enuPolePoint.z = 0;
    pub.publish(enuPolePoint);
    hasPole = false;
  }
}

// callback signature, assuming your points are pcl::PointXYZ type:
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& input){
  //ROS_INFO("Point Cloud 2 Callback");  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr filteredGround,filteredBackground,pole;
  sensor_msgs::PointCloud2 test,line;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    filteredGround(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    filteredBackground(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    pole(new pcl::PointCloud<pcl::PointXYZ>());

  sensor_msgs::PointCloud2 output;
  //ROS_INFO("Converting point clouds");
  pcl::fromROSMsg(*input,cloud);

  //First transform the point cloud
  redStereo->transform(cloud.makeShared());

  //Then filter out ground
  //ROS_INFO("Filtering background");
  redStereo->filterBackground(cloud.makeShared(),filteredGround);
  //Then filter out background
  //ROS_INFO("Filtering ground");
  redStereo->filterGround(filteredGround,filteredBackground);
  pcl::toROSMsg(*filteredBackground,test);
  test.header = input->header;
  test_pub.publish(test);
  //Finally obtain pole location
  //ROS_INFO("Locating Pole");
  bool found_pole = redStereo->findPole(filteredBackground,pole);
  if(found_pole){
    pcl::toROSMsg(*pole,line);
    line.header = input->header;
    line_pub.publish(line);
       
    //ROS_INFO("Condensing Pole into Pole");
    //Obtain the pole point in the Bumblebee reference frame
    redStereo->cloud2point(pole,localPolePoint);
    hasPole = true;
    //pub.publish(polePoint);   
  }else{
    //ROS_INFO("No Pole Found");
  }
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
  nh.param("ground_height", ground_height, -0.8);
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
  
  // create a templated publisher
  pub = nh.advertise<geometry_msgs::Point> (pole_namespace, queue_size);
  line_pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo_camera/line", queue_size);
  test_pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo_camera/test", queue_size);
  ros::spin();
}
