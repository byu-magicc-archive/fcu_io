#include <ros/ros.h>
#include "fcu_io/fcu_io.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "fcu_io_node");
  ros::NodeHandle nh;

  fcu_io::fcuIO Naze32;

  ros::spin();

  return 0;
}
