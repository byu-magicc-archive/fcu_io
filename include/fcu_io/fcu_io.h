#ifndef FCU_IO_H
#define FCU_IO_H

#include <ros/ros.h>
#include <fcu_io/Command.h>
#include <fcu_io/GainConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <naze_ros/Command.h>
#include <naze_ros/GPS.h>

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

namespace fcu_io
{

typedef enum {
//    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 0, // almost always present
    SENSOR_BARO = 1 << 1,
    SENSOR_MAG = 1 << 2,
    SENSOR_GPS = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_AIRSPEED = 1 << 5,
} sensors_e;


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


class fcuIO
{

public:

  fcuIO();
  ~fcuIO();
  void imuCallback(const ros::TimerEvent& event);
  void rcCallback(const ros::TimerEvent& event);

  void asCallback(const ros::TimerEvent& event);
  void altCallback(const ros::TimerEvent& event);
  void sonarCallback(const ros::TimerEvent& event);
  void RPYCallback(const fcu_io::CommandConstPtr &msg);
  void calibrationCallback(const std_msgs::BoolConstPtr &msg);
  void gpsCallback(const ros::TimerEvent& event);

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;         //!< public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; //!< private node handle for pulling parameter values from the parameter server

  // Publishers, Subscribers and Timers
  ros::Subscriber Command_subscriber_;
  ros::Subscriber RC_calibration_subscriber_;

  ros::Publisher Imu_publisher_;
  ros::Publisher Mag_publisher_;
  ros::Publisher Sonar_publisher_;
  ros::Publisher Baro_alt_publisher_;
  ros::Publisher Airspeed_publisher_;

  ros::Timer imu_pub_timer_;
  ros::Timer rc_send_timer_;
  ros::Timer as_pub_timer_;
  ros::Timer alt_pub_timer_;
  ros::Timer sonar_pub_timer_;

  // Gain controller
  dynamic_reconfigure::Server<fcu_io::GainConfig> server_;
  dynamic_reconfigure::Server<fcu_io::GainConfig>::CallbackType func_;
  void gainCallback(fcu_io::GainConfig &config, uint32_t level);

  // Parameters
  int min_PWM_output_;
  int max_PWM_output_;
  double max_roll_;
  double max_pitch_;
  double max_yaw_rate_;
  double max_throttle_;
  bool get_imu_attitude_;

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
  bool have_mag_;

  // Functions
  bool getImu();
  bool sendRC();
  bool getRC();
  bool getAS();
  bool calibrateIMU();
  bool calibrateRC();
  bool loadRCFromParam();
  bool getPID();
  bool getGPS();
  bool getStatus(uint16_t &sensors, int &cycle_time, int &i2c_errors);
  void getAttitude(geometry_msgs::Quaternion &orientation);
  bool setPID(PIDitem roll, PIDitem pitch, PIDitem yaw);

  int sat(int input, int min, int max);
};

} // namespace fcu_io

#endif // FCU_IO_H
