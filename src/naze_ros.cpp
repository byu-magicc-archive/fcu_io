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
  nh_private_.param<int>("baudrate", baudrate, 230400);
  nh_private_.param<double>("timeout", timeout_, 10);


  // connect serial port
  serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_);
  MSP_ = new MSP(serial_port, (u_int32_t)baudrate, timeout);

  // Set up Callbacks
  Command_subscriber_ = nh_.subscribe("command", 1, &nazeROS::RPYCallback, this);
  imu_pub_timer_ = nh_.createTimer(ros::Duration(1.0/imu_pub_rate), &nazeROS::imuCallback, this);
  rc_send_timer_ = nh_.createTimer(ros::Duration(1.0/rc_send_rate), &nazeROS::rcCallback, this);

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
  boost::array<double,9> lin_covariance = { {0.0004, 0.0, 0.0,
                                             0.0, 0.0004, 0.0,
                                             0.0, 0.0, 0.0004} };
  boost::array<double,9> ang_covariance = { {0.05*M_PI/180.0, 0.0, 0.0,
                                             0.0, 0.05*M_PI/180.0, 0.0,
                                             0.0, 0.0, 0.05*M_PI/180.0} };

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
  double command[4] = {0.0, 0.0, 0.0, 0.0};
  uint16_t PWM_range;
  command[RC_AIL] = msg->roll/max_roll_;
  command[RC_ELE] = msg->pitch/max_pitch_;
  command[RC_THR] = msg->thrust/max_throttle_;
  command[RC_RUD] = msg->yaw_rate/max_yaw_rate_;

  for(int i=0; i<8; i++){
    if(i == RC_AIL || i == RC_ELE || i == RC_RUD){
      PWM_range = max_sticks_[i] - min_sticks_[i];
      rc_commands_[i] = (uint16_t)sat((int)round(command[i]*PWM_range)+center_sticks_[i],min_PWM_output_, max_PWM_output_);
    }else if(i == RC_THR){
      PWM_range = max_sticks_[i] - min_sticks_[i];
      rc_commands_[i] = (uint16_t)sat((int)round(command[i]*PWM_range)+center_sticks_[i],min_PWM_output_, max_PWM_output_);
    }else{
      PWM_range = max_PWM_output_-min_PWM_output_;
      rc_commands_[i] = (uint16_t)(round(command[i]*PWM_range + min_PWM_output_));
    }
  }
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


bool nazeROS::calibrateRC()
{
  bool success;
  RC read_rc_commands;
  double sum[4] = {0.0, 0.0,0.0,0.0};

  ROS_WARN("Calibrating trim with RC channels, leave sticks at center, and throttle low");
  for(int i=0; i<10; i++){
    success = MSP_->getRC(read_rc_commands);
    if(success){
      for(int j=0; j<4; j++){
        sum[j] += read_rc_commands.rcData[j];
      }
    }else{
      ROS_ERROR("Serial Read Error");
    }
    usleep(20000);
  }
  ROS_WARN("Saving Centered Data");
  for(int i=0; i<4; i++){
    center_sticks_[i] = (uint16_t)round(sum[i]/10.0);
    ROS_INFO_STREAM("Channel " << i << " trim = " << center_sticks_[i]);
  }

  ROS_WARN("Calibrating maximum and minimum of RC channels, move sticks to full extent");
  uint16_t max_PWM_received[4] = {center_sticks_[0], center_sticks_[1], center_sticks_[2], center_sticks_[3]};
  uint16_t min_PWM_received[4] = {center_sticks_[0], center_sticks_[1], center_sticks_[2], center_sticks_[3]};
  for(int i=0; i<100; i++){
    success = MSP_->getRC(read_rc_commands);
    if(success){
      for(int j=0; j<4; j++){
        max_PWM_received[j] = (uint16_t)std::max(max_PWM_received[j],read_rc_commands.rcData[j]);
        min_PWM_received[j] = (uint16_t)std::min(min_PWM_received[j],read_rc_commands.rcData[j]);
      }
    }else{
      ROS_ERROR("Serial Read Error");
      success = true;
    }
    usleep(20000);
  }
  ROS_WARN("Saving Maximum and Minimum Data");
  for(int i=0; i<4; i++){
    max_sticks_[i] = max_PWM_received[i];
    min_sticks_[i] = min_PWM_received[i];
    ROS_INFO_STREAM("channel " << i << ": max = " << max_sticks_[i] << " min = " << min_sticks_[i]);
  }
  return true;
}


bool nazeROS::loadRCFromParam()
{
  nh_private_.param<int>("roll/max", max_sticks_[RC_AIL], 2000);
  nh_private_.param<int>("pitch/max", max_sticks_[RC_ELE], 2000);
  nh_private_.param<int>("yaw/max", max_sticks_[RC_RUD], 2000);
  nh_private_.param<int>("thrust/max", max_sticks_[RC_THR], 2000);

  nh_private_.param<int>("roll/min", min_sticks_[RC_AIL], 1000);
  nh_private_.param<int>("pitch/min", min_sticks_[RC_ELE], 1000);
  nh_private_.param<int>("yaw/min", min_sticks_[RC_RUD], 1000);
  nh_private_.param<int>("thrust/min", min_sticks_[RC_THR], 1000);

  nh_private_.param<int>("roll/center", center_sticks_[RC_AIL], 1500);
  nh_private_.param<int>("pitch/center", center_sticks_[RC_ELE], 1500);
  nh_private_.param<int>("yaw/center", center_sticks_[RC_RUD], 1500);
  nh_private_.param<int>("thrust/center", center_sticks_[RC_THR], 1500);
  return true;
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
    Imu_.linear_acceleration.x = (double)receivedIMUdata.accx/512.0*9.80665;
    Imu_.linear_acceleration.y = (double)receivedIMUdata.accy/512.0*9.80665;
    Imu_.linear_acceleration.z = (double)receivedIMUdata.accz/512.0*9.80665;
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
