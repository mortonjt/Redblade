#include <gtest/gtest.h>
#include "redblade_laser.h"

void arbituaryCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int numPoints){
  cloud->width = numPoints;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1;
    cloud->points[i].y = 2;
    cloud->points[i].z = 3;
  }
}

void randomCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int numPoints){
  cloud->width = numPoints;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f);
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f);
  }
}

void randomCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double centers, int numPoints){
  cloud->width = numPoints;
  cloud->height = 1;
  cloud->points.resize(cloud->width*cloud->height);
  for(size_t i = 0; i<cloud->points.size(); ++i){
    cloud->points[i].x = 1024*rand()/(RAND_MAX+1.0f) + centers;
    cloud->points[i].y = 1024*rand()/(RAND_MAX+1.0f) + centers;
    cloud->points[i].z = 1024*rand()/(RAND_MAX+1.0f) + centers;
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

TEST(redblade_laser,testProjectLaser){                       
  laser_geometry::LaserProjection projector_;
  projector_.projectLaser(currentScan, cloud); 
}

TEST(redblade_laser,testFindPole){                       
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
  EXPECT_NEAR(point.z,10.0,1);    
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
