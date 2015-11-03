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
  nh_private_.param<double>("max_yaw_rate", max_yaw_rate_, 45.0*M_PI/180.0);
  nh_private_.param<double>("max_throttle", max_throttle_, 55.0);
  nh_private_.param<std::string>("imu_frame_id", imu_frame_id_, "shredder/base/Imu");
  nh_private_.param<double>("imu_pub_rate", imu_pub_rate, 100.0);
  nh_private_.param<double>("rc_send_rate", rc_send_rate, 50.0);
  nh_private_.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  nh_private_.param<int>("baudrate", baudrate, 115200);
  nh_private_.param<double>("timeout", timeout_, 10);


  // connect serial port
  serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_);
  MSP_ = new MSP(serial_port, (u_int32_t)baudrate, timeout);

  // Set up Callbacks
  Command_subscriber_ = nh_.subscribe("command", 1, &nazeROS::RPYCallback, this);
  imu_pub_timer_ = nh_.createTimer(ros::Duration(1.0d/imu_pub_rate), &nazeROS::imuCallback, this);
//  rc_send_timer_ = nh_.createTimer(ros::Duration(1.0d/rc_send_rate), &nazeROS::rcCallback, this);

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
}

nazeROS::~nazeROS(){
  delete MSP_;
}


void nazeROS::RPYCallback(const relative_nav_msgs::CommandConstPtr &msg)
{
  rc_commands_[RC_AIL] = mapPercentToRC(msg->roll/max_roll_);
  rc_commands_[RC_ELE] = mapPercentToRC(msg->pitch/max_pitch_);
  rc_commands_[RC_THR] = mapPercentToRC(msg->roll/max_throttle_);
  rc_commands_[RC_RUD] = mapPercentToRC(msg->roll/max_roll_);
}


void nazeROS::imuCallback(const ros::TimerEvent& event)
{
  if(!getImu())
  {
    ROS_ERROR_STREAM("IMU receive error");
  }
}


//void nazeROS::rcCallback(const ros::TimerEvent& event)
//{
//  if(!sendRC())
//  {
//    ROS_ERROR_STREAM("RC Send Error");
//  }
//}


//bool nazeROS::sendRC()
//{
//  SetRawRC outgoing_rc_commands;
//  RC received_rc_commands;
//  memset(&outgoing_rc_commands, 0, sizeof(outgoing_rc_commands));
//  memset(&received_rc_commands, 0, sizeof(received_rc_commands));

//  // send new commands down the wire
//  for(int i = 0; i<8; i++){
//   outgoing_rc_commands.rcData[i] = rc_commands_[i];
//  }
////  MSP_.send(outgoing_rc_commands);
//  ros::Time t_sent = ros::Time::now();

//  bool received(false);
//  bool timeout_flag(false);
//  while(!received && !timeout_flag)
//  {
//      get<RC>(received_rc_commands);
//      received = true;
//      for(int i=0; i<8;i++){
//        if(received_rc_commands.rcData[i] != outgoing_rc_commands.rcData[i])
//          received = false;
//      }
//      if((ros::Time::now() - t_sent).toSec() > timeout_){
//        timeout_flag = true;
//      }
//  }
//  return !timeout_flag || received;
//}


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


uint16_t nazeROS::mapPercentToRC(double percent_command)
{
  percent_command*(max_PWM_output_-min_PWM_output_) + min_PWM_output_;
}

} // namespace naze_ros
