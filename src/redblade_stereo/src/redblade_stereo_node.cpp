#include "redblade_stereo.h"

ros::Publisher pub;
ros::Publisher test_pub;
ros::Publisher line_pub;
redblade_stereo* redStereo;

// callback signature, assuming your points are pcl::PointXYZ type:
void callback(const sensor_msgs::PointCloud2ConstPtr& input){
  ROS_INFO("Point Cloud 2 Callback");  
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
  geometry_msgs::Point polePoint;
  ROS_INFO("Converting point clouds");
  pcl::fromROSMsg(*input,cloud);
  
  //First filter out ground
  ROS_INFO("Filtering background");
  redStereo->filterBackground(cloud.makeShared(),filteredGround);
  //Then filter out background
  ROS_INFO("Filtering ground");
  redStereo->filterGround(filteredGround,filteredBackground);
  pcl::toROSMsg(*filteredBackground,test);
  test.header = input->header;
  test_pub.publish(test);
  //Finally obtain pole location
  ROS_INFO("Locating Pole");
  bool found_pole = redStereo->findPole(filteredBackground,pole);
  if(found_pole){
    pcl::toROSMsg(*pole,line);
    line.header = input->header;
    line_pub.publish(line);
       
    ROS_INFO("Condensing Pole into Pole");
    redStereo->cloud2point(pole,polePoint);
    pub.publish(polePoint);   
  }else{
    ROS_INFO("No Pole Found");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "redblade_stereo");
  //ros::NodeHandle n; 
  ros::NodeHandle nh;
  std::string stereo_namespace,pole_namespace;
  int queue_size;
  double height,radius,width;
  std::string topic;

  //See odometry_skid_steer.h for all constants
  nh.param("queue_size", queue_size, 2);
  nh.param("stereo_namespace", stereo_namespace, std::string("/stereo_camera/points2"));
  nh.param("pole_namespace", pole_namespace, std::string("/pole"));
  nh.param("ground_height", height, -0.8);
  nh.param("viewing_radius", radius, 9.5);
  nh.param("pole_width", width, 0.01);
  ROS_INFO("Stereo Namespace %s",stereo_namespace.c_str());
  ROS_INFO("Ground Height %f Viewing Radius %f",height,radius);
  
  //redblade_stereo redStereo(radius,height,width);
  redStereo = new redblade_stereo(radius,height,width);
  
  // create a templated subscriber
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (stereo_namespace, queue_size, callback);
  
  // create a templated publisher
  pub = nh.advertise<geometry_msgs::Point> (pole_namespace, queue_size);
  line_pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo_camera/line", queue_size);
  test_pub = nh.advertise<sensor_msgs::PointCloud2> ("/stereo_camera/test", queue_size);
  ros::spin();
}
