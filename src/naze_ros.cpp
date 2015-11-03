#include "naze_ros/naze_ros.h"

namespace naze_ros
{

nazeROS::nazeROS() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // retrieve params
  double imu_pub_rate, rc_send_rate;
  std::string serial_port;
  int baudrate;
  nh_private_.param<int>("min_PWM_output", min_PWM_output_, 1000);
  nh_private_.param<int>("max_PWM_output", max_PWM_output_, 2000);
  nh_private_.param<double>("max_roll", max_roll_, 45.0*M_PI/180.0);
  nh_private_.param<double>("max_pitch", max_pitch_, 45.0*M_PI/180.0);
  nh_private_.param<double>("max_yaw_rate", max_yaw_rate_, M_PI);
  nh_private_.param<double>("max_throttle", max_throttle_, 74.0);
  nh_private_.param<std::string>("imu_frame_id", imu_frame_id_, "shredder/base/Imu");
  nh_private_.param<double>("imu_pub_rate", imu_pub_rate, 100.0);
  nh_private_.param<double>("rc_send_rate", rc_send_rate, 1.0);
  nh_private_.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  nh_private_.param<int>("baudrate", baudrate, 115200);
  nh_private_.param<double>("timeout", timeout_, 10);


  // connect serial port
  serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_);
  MSP_ = new MSP(serial_port, (u_int32_t)baudrate, timeout);

  // Set up Callbacks
  Command_subscriber_ = nh_.subscribe("command", 1, &nazeROS::RPYCallback, this);
  imu_pub_timer_ = nh_.createTimer(ros::Duration(1.0d/imu_pub_rate), &nazeROS::imuCallback, this);
  rc_send_timer_ = nh_.createTimer(ros::Duration(1.0d/rc_send_rate), &nazeROS::rcCallback, this);

  // setup publishers
  Imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);

  // initialize variables
  rc_commands_[RC_AIL] = 0;
  rc_commands_[RC_ELE] = 0;
  rc_commands_[RC_THR] = 0;
  rc_commands_[RC_RUD] = 0;
  rc_commands_[RC_AUX1] = 0;
  rc_commands_[RC_AUX2] = 0;
  rc_commands_[RC_AUX3] = 0;
  rc_commands_[RC_AUX4] = 0;

  //initialize constant message members
  Imu_.header.frame_id = imu_frame_id_;
  // accel noise is 400 ug's
  // gyro noise is 0.05 deg/s
  boost::array<double,9> lin_covariance = { {0.0004d, 0.0d, 0.0d,
                                             0.0d, 0.0004d, 0.0d,
                                             0.0d, 0.0d, 0.0004d} };
  boost::array<double,9> ang_covariance = { {0.05*M_PI/180.0, 0.0d, 0.0d,
                                             0.0d, 0.05*M_PI/180.0, 0.0d,
                                             0.0d, 0.0d, 0.05*M_PI/180.0} };

  Imu_.angular_velocity_covariance = ang_covariance;
  Imu_.linear_acceleration_covariance = lin_covariance;

  if(calibrateIMU()){
    ROS_INFO("IMU calibration successful");
  }else{
    ROS_ERROR("IMU calibration unsuccessful");
  }
}

nazeROS::~nazeROS(){
  delete MSP_;
}


void nazeROS::RPYCallback(const relative_nav_msgs::CommandConstPtr &msg)
{
  int aux1(0.0), aux2(0.0), aux3(0.0), aux4(0.0);
  rc_commands_[RC_AIL] = (u_int16_t)sat(mapPercentToRC((msg->roll/max_roll_)+0.5d), min_PWM_output_, max_PWM_output_);
  rc_commands_[RC_ELE] = (u_int16_t)sat(mapPercentToRC((msg->pitch/max_pitch_)+0.5d), min_PWM_output_, max_PWM_output_);
  rc_commands_[RC_THR] = (u_int16_t)sat(mapPercentToRC((msg->thrust/max_throttle_)), min_PWM_output_, max_PWM_output_);
  rc_commands_[RC_RUD] = (u_int16_t)sat(mapPercentToRC((msg->yaw_rate/max_yaw_rate_)+0.5d), min_PWM_output_, max_PWM_output_);
  rc_commands_[RC_AUX1] = (u_int16_t)sat(mapPercentToRC(aux1), min_PWM_output_, max_PWM_output_);
  rc_commands_[RC_AUX2] = (u_int16_t)sat(mapPercentToRC(aux2), min_PWM_output_, max_PWM_output_);
  rc_commands_[RC_AUX3] = (u_int16_t)sat(mapPercentToRC(aux3), min_PWM_output_, max_PWM_output_);
  rc_commands_[RC_AUX4] = (u_int16_t)sat(mapPercentToRC(aux4), min_PWM_output_, max_PWM_output_);
}


void nazeROS::imuCallback(const ros::TimerEvent& event)
{
  if(!getImu()){
    ROS_ERROR_STREAM("IMU receive error");
  }
}


void nazeROS::rcCallback(const ros::TimerEvent& event)
{
  if(!sendRC()){
    ROS_ERROR_STREAM("RC Send Error");
  }
}

bool nazeROS::calibrateIMU()
{
  MSP_->calibrateIMU();
}


bool nazeROS::sendRC()
{
  SetRawRC outgoing_rc_commands;
  for(int i=0; i<8; i++){
    outgoing_rc_commands.rcData[i] = rc_commands_[i];
  }
  MSP_->setRawRC(outgoing_rc_commands);
  return getRC();
}

bool nazeROS::getRC()
{
  RC actual_rc_commands;
  bool success = MSP_->getRC(actual_rc_commands);
  ROS_INFO_STREAM("RC Commands:");
  for(int i=0; i<8; i++){
    ROS_INFO_STREAM("i: = " << actual_rc_commands.rcData[i]);
  }
  return true;
}


bool nazeROS::getImu()
{
  RawIMU receivedIMUdata;
  memset(&receivedIMUdata, 0, sizeof(receivedIMUdata));

  bool received = MSP_->getRawIMU(receivedIMUdata);
  if(received){
    Imu_.linear_acceleration.x = (double)receivedIMUdata.accx/512.0;
    Imu_.linear_acceleration.y = (double)receivedIMUdata.accy/512.0;
    Imu_.linear_acceleration.z = (double)receivedIMUdata.accz/512.0;
    Imu_.angular_velocity.x = (double)receivedIMUdata.gyrx*4.096/180.0*M_PI;
    Imu_.angular_velocity.y = (double)receivedIMUdata.gyry*4.096/180.0*M_PI;
    Imu_.angular_velocity.z = (double)receivedIMUdata.gyrz*4.096/180.0*M_PI;
    Imu_.header.stamp = ros::Time::now();
    Imu_publisher_.publish(Imu_);
    return true;
  } else{
    return false;
  }
}


int nazeROS::mapPercentToRC(double percent_command)
{
  int output =  (int)round(percent_command*(max_PWM_output_-min_PWM_output_)) + min_PWM_output_;
//  ROS_INFO_STREAM("input " << percent_command << " output " << output);
}

int nazeROS::sat(int input, int min, int max){
  int output(input);
  if(input > max){
    output = max;
  }else if(input < min){
    output = min;
  }
  return output;
}

} // namespace naze_ros
