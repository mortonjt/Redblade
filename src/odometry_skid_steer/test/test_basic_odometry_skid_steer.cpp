#include "odometry_skid_steer.h"
#include <gtest/gtest.h>

// namespace{
//   class test_odometry_skid_steer : public ::testing::Test{
//   protected:
//     // You can remove any or all of the following functions if its body
//     // is empty.

//     FooTest() {
//       // You can do set-up work for each test here.
      
//     }

//     virtual ~FooTest() {
//       // You can do clean-up work that doesn't throw exceptions here.
//     }

//     // If the constructor and destructor are not enough for setting up
//     // and cleaning up each test, you can define the following methods:

//     virtual void SetUp() {
//       // Code here will be called immediately after the constructor (right
//       // before each test).
//     }

//     virtual void TearDown() {
//       // Code here will be called immediately after each test (right
//       // before the destructor).
//     }

//     // Objects declared here can be used by all tests in the test case for Foo.
//   };


//The actual unittests
// TEST(odometry_skid_steer, testDeltaAnglePos){
//   odometry_skid_steer testOdomSS(0,0);
//   ros::Time now(0);
//   redblade_ax2550::StampedEncoders front_encoder_msg;
//   front_encoder_msg.header.stamp = now;
//   front_encoder_msg.header.frame_id = "base_link";
//   front_encoder_msg.encoders.time_delta = 0.01;
//   front_encoder_msg.encoders.left_wheel = 10;
//   front_encoder_msg.encoders.right_wheel = -10;
//   redblade_ax2550::StampedEncoders back_encoder_msg;
//   back_encoder_msg.header.stamp = now;
//   back_encoder_msg.header.frame_id = "base_link";
//   back_encoder_msg.encoders.time_delta = 0.01;
//   back_encoder_msg.encoders.left_wheel = 10;
//   back_encoder_msg.encoders.right_wheel = -10;
//   geometry_msgs::Vector3 orientation_msg; 
//   orientation_msg.x = 0;
//   orientation_msg.y = 0;
//   orientation_msg.z = 0;
//   std::cout<<"Front time delta "<<front_encoder_msg.encoders.time_delta<<std::endl;
//   std::cout<<"Back time delta "<<back_encoder_msg.encoders.time_delta<<std::endl;
//   double delta_time;
//   double distance_delta;
//   double theta_delta;
//   testOdomSS.getDeltaAnglePos(front_encoder_msg,
// 			      back_encoder_msg,
// 			      orientation_msg,
// 			      delta_time,
// 			      distance_delta,
// 			      theta_delta);
//   EXPECT_NEAR(0.01,delta_time,0.0001);
//   EXPECT_NEAR(10/clicks_per_m,distance_delta,0.0001);
//   EXPECT_NEAR(0,theta_delta,0.0001);
// }  
//}

//The actual unittests
TEST(basic_odometry_skid_steer, testEncoders){
  double wheel_base_width = 1.0;
  odometry_skid_steer testOdomSS("odom_frame",0,0,wheel_base_width);
  ros::Time now(0);

  redblade_ax2550::StampedEncoders front_encoder_msg;
  front_encoder_msg.header.stamp = now;
  front_encoder_msg.header.frame_id = "base_link";
  front_encoder_msg.encoders.time_delta = 0.01;
  front_encoder_msg.encoders.left_wheel = 10;
  front_encoder_msg.encoders.right_wheel = -10;

  redblade_ax2550::StampedEncoders back_encoder_msg;
  back_encoder_msg.header.stamp = now;
  back_encoder_msg.header.frame_id = "base_link";
  back_encoder_msg.encoders.time_delta = 0.01;
  back_encoder_msg.encoders.left_wheel = 10;
  back_encoder_msg.encoders.right_wheel = -10;

  geometry_msgs::Vector3 orientation_msg; 
  orientation_msg.x = 0;
  orientation_msg.y = 0;
  orientation_msg.z = 0;
 
  std::cout<<"Front time delta "<<front_encoder_msg.encoders.time_delta<<std::endl;
  std::cout<<"Back time delta "<<back_encoder_msg.encoders.time_delta<<std::endl;

  double distance_delta;
  double theta_delta;
  geometry_msgs::Twist twist_vel;
  double delta_time,left_encoders,right_encoders;
  
  testOdomSS.getEncoders(front_encoder_msg,back_encoder_msg,left_encoders,right_encoders,delta_time);
  EXPECT_NEAR(10,left_encoders,0.0001);
  EXPECT_NEAR(10,right_encoders,0.0001);
  EXPECT_NEAR(0.01,delta_time,0.0001);
}


TEST(basic_odometry_skid_steer, testVelocity){
  double wheel_base_width = 1.0;
  odometry_skid_steer testOdomSS("odom_frame",0,0,wheel_base_width);
  ros::Time now(0);

  redblade_ax2550::StampedEncoders front_encoder_msg;
  front_encoder_msg.header.stamp = now;
  front_encoder_msg.header.frame_id = "base_link";
  front_encoder_msg.encoders.time_delta = 0.01;
  front_encoder_msg.encoders.left_wheel = 10;
  front_encoder_msg.encoders.right_wheel = -10;

  redblade_ax2550::StampedEncoders back_encoder_msg;
  back_encoder_msg.header.stamp = now;
  back_encoder_msg.header.frame_id = "base_link";
  back_encoder_msg.encoders.time_delta = 0.01;
  back_encoder_msg.encoders.left_wheel = 10;
  back_encoder_msg.encoders.right_wheel = -10;

  geometry_msgs::Vector3 orientation_msg; 
  orientation_msg.x = 0;
  orientation_msg.y = 0;
  orientation_msg.z = 0;
 
  std::cout<<"Front time delta "<<front_encoder_msg.encoders.time_delta<<std::endl;
  std::cout<<"Back time delta "<<back_encoder_msg.encoders.time_delta<<std::endl;

  double distance_delta;
  double theta_delta;
  geometry_msgs::Twist twist_vel;
  double delta_time,left_encoders,right_encoders;
  
  testOdomSS.getEncoders(front_encoder_msg,back_encoder_msg,left_encoders,right_encoders,delta_time);
  testOdomSS.getVelocities(front_encoder_msg,
			   back_encoder_msg,
			   twist_vel);  
  EXPECT_NEAR((10/clicks_per_m)/delta_time,twist_vel.linear.x,0.0001);
  EXPECT_GT(100,twist_vel.angular.z);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
