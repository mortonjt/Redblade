#include <gtest/gtest.h>
#include "redblade_laser.h"

void arbituaryCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int numPoints){
  cloud->width = numPoints;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1;
    cloud->points[i].y = 2;
    cloud->points[i].z = 0;
  }
}

void randomCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int numPoints){
  cloud->width = numPoints;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 0;
  }
}

void randomCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double centers, int numPoints){
  cloud->width = numPoints;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f) + centers;
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f) + centers;
    cloud->points[i].z = 0;
  }
}


TEST(redblade_laser,testAddScan){
  std::string surveyFile = "test_survey.txt";
  double offset = 0.1;
  int queueSize = 3;
  std::ofstream h((char*)surveyFile.c_str());
  h<<0 <<'\t'<<0 <<std::endl;
  h<<10<<'\t'<<0 <<std::endl;
  h<<0 <<'\t'<<10<<std::endl;
  h<<10<<'\t'<<10<<std::endl;
  h.close();
  redblade_laser testLazer(surveyFile,offset,queueSize);  
  std::remove("test_survey.txt");
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud2(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud3(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud4(new pcl::PointCloud<pcl::PointXYZ>());
  arbituaryCloud(cloud1,1);
  arbituaryCloud(cloud2,3);
  arbituaryCloud(cloud3,5);
  arbituaryCloud(cloud4,7);
  testLazer.addScan(cloud1);
  EXPECT_EQ(testLazer.queue.size(),1);
  testLazer.addScan(cloud2);
  testLazer.addScan(cloud3);
  EXPECT_EQ(testLazer.queue.size(),3);
  testLazer.addScan(cloud4);
  EXPECT_EQ(testLazer.queue.size(),3);
}

 TEST(redblade_laser,testGetClouds){
  std::string surveyFile = "test_survey.txt";
  double offset = 0.1;
  int queueSize = 3;
  std::ofstream h((char*)surveyFile.c_str());
  h<<0 <<'\t'<<0 <<std::endl;
  h<<10<<'\t'<<0 <<std::endl;
  h<<0 <<'\t'<<10<<std::endl;
  h<<10<<'\t'<<10<<std::endl;
  h.close();
  redblade_laser testLazer(surveyFile,offset,queueSize);  
  std::remove("test_survey.txt");
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud2(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud3(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud4(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    result(new pcl::PointCloud<pcl::PointXYZ>());
  
  arbituaryCloud(cloud1,1);
  arbituaryCloud(cloud2,3);
  arbituaryCloud(cloud3,5);
  arbituaryCloud(cloud4,7);
  testLazer.addScan(cloud1);
  testLazer.getClouds(result);
  EXPECT_EQ(result->points.size(),1);
  testLazer.addScan(cloud2);
  testLazer.addScan(cloud3);
  testLazer.getClouds(result);
  EXPECT_EQ(result->points.size(),9);
  testLazer.addScan(cloud4);
  testLazer.getClouds(result);
  EXPECT_EQ(result->points.size(),15);
}
TEST(redblade_laser,testInBounds1){
  std::string surveyFile = "test_survey.txt";
  double offset = 0.1;
  int queueSize = 3;
  std::ofstream h((char*)surveyFile.c_str());
  h<<0 <<'\t'<<0 <<std::endl;
  h<<0<<'\t'<<zoneLength <<std::endl;
  h.close();
  bool searchSnowField = true;
  redblade_laser testLazer(surveyFile,searchSnowField,offset,queueSize);
  double x = 0; double y = zoneLength;
  testLazer.rotate(x,y);
  EXPECT_NEAR(x,zoneLength,0.01);
  EXPECT_NEAR(y,0,0.01);
  x = -zoneWidth;  y = zoneLength;
  testLazer.rotate(x,y);
  EXPECT_NEAR(x,zoneLength,0.01);
  EXPECT_NEAR(y,zoneWidth,0.01);
  x = -zoneWidth;  y = 0;
  testLazer.rotate(x,y);
  EXPECT_NEAR(x,0,0.01);
  EXPECT_NEAR(y,zoneWidth,0.01);
  
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 2; cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  cloud->points[0].x = -2;
  cloud->points[0].y = 4;
  cloud->points[0].z = 0;
  EXPECT_TRUE(testLazer.inBounds(cloud->points[0].x,
				 cloud->points[0].y));
  cloud->points[1].x = -1;
  cloud->points[1].y = 4;
  cloud->points[1].z = 0;
  EXPECT_FALSE(testLazer.inBounds(cloud->points[1].x,
				  cloud->points[1].y));
    
}

TEST(redblade_laser,testInBounds2){
  std::string surveyFile = "test_survey.txt";
  double offset = 0.1;
  int queueSize = 3;
  std::ofstream h((char*)surveyFile.c_str());
  h<<0 <<'\t'<<0 <<std::endl;
  h<<0<<'\t'<<zoneLength <<std::endl;
  h.close();
  bool searchSnowField = false;
  redblade_laser testLazer(surveyFile,searchSnowField,offset,queueSize);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->width = 2; cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  cloud->points[0].x = -2;
  cloud->points[0].y = 4;
  cloud->points[0].z = 0;
  EXPECT_FALSE(testLazer.inBounds(cloud->points[0].x,
				 cloud->points[0].y));
  cloud->points[1].x = -1;
  cloud->points[1].y = 4;
  cloud->points[1].z = 0;
  EXPECT_TRUE(testLazer.inBounds(cloud->points[1].x,
				 cloud->points[1].y));    
}

TEST(redblade_laser,testFindPole){                       
  std::string surveyFile = "test_survey.txt";
  double offset = 0.1;
  int queueSize = 3;
  std::ofstream h((char*)surveyFile.c_str());
  h<<0 <<'\t'<<0 <<std::endl;
  h<<10<<'\t'<<0 <<std::endl;
  h<<0 <<'\t'<<zoneLength<<std::endl;
  h<<10<<'\t'<<zoneLength<<std::endl;
  h.close();
  redblade_laser testLazer(surveyFile,offset,queueSize);  
  std::remove("test_survey.txt");
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud2(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud3(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    cloud4(new pcl::PointCloud<pcl::PointXYZ>());
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > 
    result(new pcl::PointCloud<pcl::PointXYZ>());
  geometry_msgs::Point point;
  randomCloud(cloud1,0,1);
  randomCloud(cloud2,0,3);
  randomCloud(cloud3,10,5);
  randomCloud(cloud4,10,7);
  testLazer.addScan(cloud1);
  testLazer.addScan(cloud2);
  testLazer.addScan(cloud3);
  testLazer.addScan(cloud4);
  testLazer.findPole(point,1.0);
  EXPECT_NEAR(point.x,10.0,1);  
  EXPECT_NEAR(point.y,10.0,1);    
  EXPECT_NEAR(point.z,0,1);    
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
