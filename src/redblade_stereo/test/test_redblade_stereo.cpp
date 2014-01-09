#include <gtest/gtest.h>
#include "redblade_stereo.h"

TEST(redblade_stereo,testFilterGround1){
  double radius = 1.0;
  double groundHeight = 0.0;
  double width = 0.05;
  double cameraHeight = 1.0;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    above(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 100;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f);
  }
  redblade_stereo testRS(radius,0.0,groundHeight,width,cameraHeight,0.0);
  testRS.filterGround(cloud,above);
  EXPECT_GT(above->points.size(),0);
  for(size_t i = 0; i<above->points.size();i++){
    EXPECT_GT(above->points[i].z,groundHeight);
  }
}

TEST(redblade_stereo,testFilterRadius1){
  double radius = 1.0;
  double groundHeight = 0.0;
  double width = 0.05;
  double cameraHeight = 1.0;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    above(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 100;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f);
  }
  redblade_stereo testRS(radius,1000.0,groundHeight,width,cameraHeight,0.0);
  testRS.filterRadius(cloud,above);
  EXPECT_GT(above->points.size(),0);
  for(size_t i = 0; i<above->points.size();i++){    
    double distance =					\
      sqrt((above->points[i].x*above->points[i].x)+	\
	   (above->points[i].y*above->points[i].y)+	\
	   (above->points[i].z*above->points[i].z));    
    EXPECT_LT(distance,2*radius);
  }
}

TEST(redblade_stereo,testFilterBackground1){
  double radius = 1.0;
  double groundHeight = 0.0;
  double width = 0.05;
  double cameraHeight = 1.0;
  std::string surveyFile = "test_survey.txt";
  std::ofstream h((char*)surveyFile.c_str());
  h<<0 <<'\t'<<0 <<std::endl;
  h<<10<<'\t'<<0 <<std::endl;
  h<<0 <<'\t'<<10<<std::endl;
  h<<10<<'\t'<<10<<std::endl;
  h.close();
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    above(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 100;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f);
  }  
  geometry_msgs::Pose2D pose;
  pose.x = 0;
  pose.y = 0;
  pose.theta = 0;
  redblade_stereo testRS(surveyFile,groundHeight,width,cameraHeight,0.0);
  testRS.filterBackground(pose,cloud,above);
  for(size_t i = 0; i<above->points.size();i++){
    EXPECT_LT(above->points[i].x,10);
    EXPECT_GT(above->points[i].x,0);
    EXPECT_LT(above->points[i].y,10);
    EXPECT_GT(above->points[i].y,0);
  }
}


TEST(redblade_stereo,testRansac){
  double radius = 1.0;
  double groundHeight = 0.0;
  double width = 0.05;
  double cameraHeight = 1.0;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::VectorXf coeff;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,line;
  cloud->width = 100;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){//Place everything on a line
    cloud->points[i].x = 1;
    cloud->points[i].y = 1;
    cloud->points[i].z = i+10;
  }
  redblade_stereo testRS(radius,0.0,groundHeight,width,cameraHeight,0.0);
  testRS.ransac(cloud,line,coeff);
  EXPECT_EQ(cloud->points.size(),line->points.size());
}


TEST(redblade_stereo,testRansac2){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.05;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::VectorXf coeff;
  cloud->width = 100;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<50; ++i){//Place straight line
    cloud->points[i].x = 0;
    cloud->points[i].y = 0;
    cloud->points[i].z = i+10;    
  }
  for(size_t i = 50; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f);
  }
  redblade_stereo testRS(radius,0.0,height,width,0.0,0.0);
  testRS.ransac(cloud,line,coeff);
  EXPECT_GT(line->points.size(),49);
}


TEST(redblade_stereo,testFindPole1){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.05;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 300;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<200; ++i){//Place straight line
    cloud->points[i].x = 0;
    cloud->points[i].y = i+10;
    cloud->points[i].z = 0;    
  }
  for(size_t i = 200; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f);
  }
  redblade_stereo testRS(radius,0.0,height,width,0.0,0.0);
  testRS.findPole(cloud,line);
  EXPECT_GT(line->points.size(),150);
}

TEST(redblade_stereo,testFindPole2){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.05;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 300;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 300; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f);
  }
  redblade_stereo testRS(radius,0.0,height,width,0.0,0.0);
  bool result = testRS.findPole(cloud,line);
  EXPECT_FALSE(result);
}

TEST(redblade_stereo,testFindPole3){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.05;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 300;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<50; ++i){//Place straight line
    cloud->points[i].x = 0;
    cloud->points[i].y = 0;
    cloud->points[i].z = i+10;    
  }
  for(size_t i = 50; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f);
  }
  redblade_stereo testRS(radius,0.0,height,width,0.0,0.0);
  bool result = testRS.findPole(cloud,line);
  EXPECT_FALSE(result);
}

TEST(redblade_stereo,testFindPole4){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.05;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 300;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){//Place straight line
    cloud->points[i].x = 0;
    cloud->points[i].y = 0;
    cloud->points[i].z = ((float)i)/1000.0+10;    
  }
  redblade_stereo testRS(radius,0.0,height,width,0.0,0.0);
  bool result = testRS.findPole(cloud,line);
  EXPECT_TRUE(result);
}

TEST(redblade_stereo,testFindPole5){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.05;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 300;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<150; ++i){//Place straight line
    cloud->points[i].x = 0;
    cloud->points[i].y = ((float)i)/1000.0+10;
    cloud->points[i].z = 0;    
  }

  for(size_t i = 150; i<cloud->points.size(); ++i){//Place straight line
    cloud->points[i].x = 0;
    cloud->points[i].y = ((float)i)/1000.0+1000;
    cloud->points[i].z = 0;   
  }

  redblade_stereo testRS(radius,0.0,height,width,0.0,0.0);
  bool result = testRS.findPole(cloud,line);
  EXPECT_FALSE(result);
}


TEST(redblade_stereo,testCluster1){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.05;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 300;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  
  for(size_t i = 0; i<100; ++i){//Centered around (0,0,0)
    cloud->points[i].x = rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = rand()/(RAND_MAX+1.0f);
  }
  for(size_t i = 100; i<cloud->points.size(); ++i){//Centered around (10,10,10)
    cloud->points[i].x = rand()/(RAND_MAX+1.0f)+10;
    cloud->points[i].y = rand()/(RAND_MAX+1.0f)+10;
    cloud->points[i].z = rand()/(RAND_MAX+1.0f)+10;
  }
  redblade_stereo testRS(radius,0.0,height,width,0.0,0.0);
  int clusters = testRS.cluster(cloud,1.0);
  EXPECT_EQ(clusters,2);  
}

TEST(redblade_stereo,testCluster2){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.05;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    line(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 300;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  
  for(size_t i = 0; i<300; ++i){//Centered around (0,0,0)
    cloud->points[i].x = rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = rand()/(RAND_MAX+1.0f);
  }
  redblade_stereo testRS(radius,0.0,height,width,0.0,0.0);
  int clusters = testRS.cluster(cloud,1.0);
  EXPECT_EQ(clusters,1);  
}

TEST(redblade_stereo,transformStereo2RobotTest1){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.01;
  double cameraHeight = 10.0;
  double cameraOffset = 1.0;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 1;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  //Camera coordinates
  cloud->points[0].x = 0; 
  cloud->points[0].y = 0; 
  cloud->points[0].z = 0; 
  redblade_stereo testRS(radius,0.0,height,width,cameraHeight,cameraOffset);
  testRS.transformStereo2Robot(cloud);
  EXPECT_EQ(cloud->points[0].z,cameraHeight);  
  EXPECT_EQ(cloud->points[0].x,cameraOffset);    
  EXPECT_EQ(cloud->points[0].y,0.0);    
}

TEST(redblade_stereo,transformRobot2ENUTest1){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.01;
  double cameraHeight = 10.0;
  double cameraOffset = 1.0;
  redblade_stereo testRS(radius,0.0,height,width,cameraHeight,cameraOffset);
  geometry_msgs::Point point,enu;
  geometry_msgs::Pose2D pose;
  pose.x = 1.0;
  pose.y = 0.0;
  pose.theta = M_PI;
  point.x = 1.0;
  point.y = 0.0;
  testRS.transformRobot2ENU(pose,point,enu);
  EXPECT_NEAR(enu.x,0.0,0.000001);  
  EXPECT_NEAR(enu.y,0.0,0.000001);    
}

TEST(redblade_stereo,transformRobot2ENUTest2){
  double radius = 1.0;
  double height = 1.0;
  double width = 0.01;
  double cameraHeight = 10.0;
  double cameraOffset = 1.0;
  redblade_stereo testRS(radius,0.0,height,width,cameraHeight,cameraOffset);
  geometry_msgs::Point point,enu;
  geometry_msgs::Pose2D pose;
  pose.x = 1.0;
  pose.y = 1.0;
  pose.theta = -M_PI/2;
  point.x = 1.0;
  point.y = 1.0;
  testRS.transformRobot2ENU(pose,point,enu);
  EXPECT_NEAR(enu.x,2.0,0.000001);  
  EXPECT_NEAR(enu.y,0.0,0.000001);    
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
