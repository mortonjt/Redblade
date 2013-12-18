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
TEST(odometry_skid_steer, testCase1){

  odometry_skid_steer testOdomSS(0,0);

  ros::Time now(0);

  ax2550::StampedEncoders front_encoder_msg;
  front_encoder_msg.header.stamp = now;
  front_encoder_msg.header.frame_id = "base_link";
  front_encoder_msg.encoders.time_delta = 0.01;
  front_encoder_msg.encoders.left_wheel = 10;
  front_encoder_msg.encoders.right_wheel = 10;

  ax2550::StampedEncoders back_encoder_msg;
  back_encoder_msg.header.stamp = now;
  back_encoder_msg.header.frame_id = "base_link";
  back_encoder_msg.encoders.time_delta = 0.01;
  back_encoder_msg.encoders.left_wheel = 10;
  back_encoder_msg.encoders.right_wheel = 10;

  double delta_time;
  double distance_delta;
  double theta_delta;

  testOdomSS.getDeltaAnglePos(front_encoder_msg,
			      back_encoder_msg,
			      delta_time,
			      distance_delta,
			      theta_delta);
  EXPECT_EQ(0.01,delta_time);
  EXPECT_EQ(10/clicks_per_m,distance_delta);
  EXPECT_EQ(0,theta_delta);
}
  
//}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
