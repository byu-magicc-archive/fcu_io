#include <ros/ros.h>
#include "naze_ros/naze_ros.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "naze_ros_node");
  ros::NodeHandle nh;

  naze_ros::nazeROS Naze32;

  ros::spin();

  return 0;
}
