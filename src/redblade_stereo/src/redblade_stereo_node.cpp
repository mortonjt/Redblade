#include "redblade_stereo.h"

ros::Publisher pub;
redblade_stereo* redStereo;

// callback signature, assuming your points are pcl::PointXYZ type:
void callback(const sensor_msgs::PointCloud2ConstPtr& input){
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,filteredGround,filteredBackground,pole;
  sensor_msgs::PointCloud2 output;
  pcl::fromROSMsg(*input,*cloud);
  //First filter out ground
  redStereo->filterGround(cloud,filteredGround);
  //Then filter out background
  redStereo->filterGround(cloud,filteredBackground);
  //Finally obtain pole location
  redStereo->findPole(cloud,pole);
  pcl::toROSMsg(*pole,output);
  pub.publish(output);   
}

int main(int argc, char** argv){
  ros::init(argc, argv, "redblade_stereo");
  ros::NodeHandle n; //in the global namespace
  ros::NodeHandle nh("~");//local namespace, used for params
  std::string stereo_namespace,pole_namespace;
  int queue_size;
  double height,radius,width;
  std::string topic;

  //See odometry_skid_steer.h for all constants
  n.param("queue_size", queue_size, 2);
  n.param("stereo_namespace", topic, std::string("/stereo_camera"));
  n.param("pole_namespace", topic, std::string("/pole"));
  n.param("ground_height", height, 0.2);
  n.param("viewing_radius", radius, 2.0);
  n.param("pole_width", width, 0.05);
  
 //redblade_stereo redStereo(radius,height,width);
 redStereo = new redblade_stereo(radius,height,width);

  // create a templated subscriber
 ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (stereo_namespace, queue_size, callback);
  
  // create a templated publisher
  pub = nh.advertise<sensor_msgs::PointCloud2> (pole_namespace, queue_size);
  ros::spin();
}
