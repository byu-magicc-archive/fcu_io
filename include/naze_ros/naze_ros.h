#ifndef NAZE_ROS_H
#define NAZE_ROS_H

#include <ros/ros.h>
#include <relative_nav_msgs/Command.h>
#include <sensor_msgs/Imu.h>

#include "serial/msp.h"
#include "serial/mspdata.h"

#define RC_AIL 0
#define RC_ELE 1
#define RC_THR 2
#define RC_RUD 3
#define RC_AUX1 4
#define RC_AUX2 5
#define RC_AUX3 6
#define RC_AUX4 7

namespace naze_ros
{

class nazeROS
{

public:

  nazeROS();
  ~nazeROS();
  void imuCallback(const ros::TimerEvent& event);
  void rcCallback(const ros::TimerEvent& event);
  void RPYCallback(const relative_nav_msgs::CommandConstPtr &msg);

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;         //!< public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; //!< private node handle for pulling parameter values from the parameter server

  // Publishers and Subscribers
  ros::Subscriber Command_subscriber_;
  ros::Publisher Imu_publisher_; 
  ros::Timer imu_pub_timer_;
  ros::Timer rc_send_timer_;

  // Parameters
  int min_PWM_output_;
  int max_PWM_output_;
  double max_roll_;
  double max_pitch_;
  double max_yaw_rate_;
  double max_throttle_;
  std::string imu_frame_id_;
  double timeout_;

  // Local Variables
  uint16_t rc_commands_[8];
  uint16_t center_sticks_[4];
  uint16_t max_sticks_[4];
  uint16_t min_sticks_[4];
  RC updated_rc_commands;
  SetRawRC outgoing_rc_commands;
  sensor_msgs::Imu Imu_;

  MSP* MSP_;

  // Functions
  bool getImu();
  bool sendRC();
  bool getRC();
  bool calibrateIMU();
  bool calibrateRC();
  bool loadRCFromParam();

  int sat(int input, int min, int max);
};

} // namespace naze_ros

#endif // nazeROS_H
