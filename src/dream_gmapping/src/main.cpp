#include "dream_gmapping/dream_gmapper.hpp"

int main(int argc, char **argv) {
  // define node's name. Then "~" gets the node's name
  ros::init(argc, argv, "dream_gmapping");
  ros::NodeHandle nh_("~");
  // separate the class from the main file so it can be unit-tested in GTest
  DreamGMapping::DreamGMapper dg(nh_);

  // For simplicity, we are using a single threaded model for subscribers
  ros::Rate r(10);
  while (!ros::isShuttingDown()) {
    dg.publish_tf();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}