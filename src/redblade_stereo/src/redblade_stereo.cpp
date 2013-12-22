#include "redblade_stereo.h"

pcl::PointCloud<pcl::PointXYZRGB> cloud;
ros::Publisher pub;

// callback signature, assuming your points are pcl::PointXYZRGB type:
void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&){
  //do some processing
}

int main(int argc, char** argv){
  ros::init(argc, argv, "redblade_stereo");
  ros::NodeHandle n; //in the global namespace
  ros::NodeHandle nh("~");//local namespace, used for params
  std::string front_encoder_namespace,back_encoder_namespace;
  double rot_cov_,pos_cov_;
  int queue_size;
  std::string topic;

  //See odometry_skid_steer.h for all constants
  n.param("queue_size", queue_size, 2);
  n.param("stereo_namespace", topic, std::string("/stereo_camera"));

  // create a templated subscriber
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > (topic, queue_size, callback);
  
  // create a templated publisher
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > (topic, queue_size);
  
  //publish cmd_vel topic every 25 ms (40 hz)
  while(ros::ok()){
    pub.publish(cloud);
  }
  return(0);

  // and just publish the object directly
}
