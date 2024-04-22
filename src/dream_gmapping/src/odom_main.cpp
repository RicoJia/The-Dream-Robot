#include "dream_gmapping/dream_odometer.hpp"
// subscriber to float32 msg
int main(int argc, char **argv) {
  // define node's name. Then "~" gets the node's name
  ros::init(argc, argv, "dream_odometer");
  ros::NodeHandle nh_("~");
  // separate the class from the main file so it can be unit-tested in GTest
  DreamGMapping::DreamOdometer dodom(nh_);

  // For simplicity, we are using a single threaded model for subscribers
  ros::spin();
  return 0;
}