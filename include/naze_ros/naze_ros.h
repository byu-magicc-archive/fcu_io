#ifndef NAZE_ROS_H
#define NAZE_ROS_H

#include <ros/ros.h>
#include <relative_nav/Command.h>
#include <naze_ros/GainConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

#include "serial/msp.h"
#include "serial/mspdata.h"

// RC channels
#define RC_AIL 0
#define RC_ELE 1
#define RC_THR 2
#define RC_RUD 3
#define RC_AUX1 4
#define RC_AUX2 5
#define RC_ARM 6
#define RC_ACRO 7

namespace naze_ros
{

struct PIDitem{
  double P;
  double I;
  double D;
  PIDitem() {
    P = 0;
    I = 0;
    D = 0;
  }
};


class nazeROS
{

public:

  nazeROS();
  ~nazeROS();
  void imuCallback(const ros::TimerEvent& event);
  void rcCallback(const ros::TimerEvent& event);
  void RPYCallback(const relative_nav::CommandConstPtr &msg);
  void calibrationCallback(const std_msgs::BoolConstPtr &msg);
  void armCallback(const std_msgs::BoolConstPtr &msg);

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;         //!< public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; //!< private node handle for pulling parameter values from the parameter server

  // Publishers and Subscribers
  ros::Subscriber Command_subscriber_;
  ros::Subscriber RC_calibration_subscriber_;
  ros::Subscriber arm_subscriber_;
  ros::Publisher Imu_publisher_;
  ros::Timer imu_pub_timer_;
  ros::Timer rc_send_timer_;

  // Gain controller
  dynamic_reconfigure::Server<naze_ros::GainConfig> server_;
  dynamic_reconfigure::Server<naze_ros::GainConfig>::CallbackType func_;
  void gainCallback(naze_ros::GainConfig &config, uint32_t level);

  // Parameters
  int min_PWM_output_;
  int max_PWM_output_;
  double max_roll_;
  double max_pitch_;
  double max_yaw_rate_;
  double max_throttle_;

  // Local Variables
  std::vector<uint16_t> rc_commands_;
  std::vector<uint16_t> center_sticks_;
  std::vector<uint16_t> max_sticks_;
  std::vector<uint16_t> min_sticks_;
  sensor_msgs::Imu Imu_;
  MSP* MSP_;
  std::vector<PIDitem> PIDs_;
  bool armed_;
  bool acro_;

  // Functions
  bool getImu();
  bool sendRC();
  bool getRC();
  bool calibrateIMU();
  bool calibrateRC();
  bool loadRCFromParam();
  bool getPID();
  bool setPID(PIDitem roll, PIDitem pitch, PIDitem yaw);

  int sat(int input, int min, int max);
};

} // namespace naze_ros

#endif // nazeROS_H
