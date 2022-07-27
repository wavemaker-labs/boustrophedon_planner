// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

// Run all the tests that were declared with TEST()
// At the same time run a ROS node with spin()
int main(int argc, char **argv){
  ros::init(argc, argv, "tester");
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle nh;

  std::thread t([]{ros::spin();});

  auto result = RUN_ALL_TESTS();

  return result;
}
