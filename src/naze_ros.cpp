#include "naze_ros/naze_ros.h"

namespace naze_ros
{

nazeROS::nazeROS() :
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  // retrieve params
  double imu_pub_rate, rc_send_rate, dtimeout;
  std::string serial_port, imu_frame_id;
  int baudrate;
  nh_private_.param<int>("min_PWM_output", min_PWM_output_, 1000);
  nh_private_.param<int>("max_PWM_output", max_PWM_output_, 2000);
  nh_private_.param<double>("max_roll", max_roll_, 25.0*M_PI/180.0);
  nh_private_.param<double>("max_pitch", max_pitch_, 25.0*M_PI/180.0);
  nh_private_.param<double>("max_yaw_rate", max_yaw_rate_, M_PI);
  nh_private_.param<double>("max_throttle", max_throttle_, 74.0);
  nh_private_.param<std::string>("imu_frame_id", imu_frame_id, "shredder/base/Imu");
  nh_private_.param<double>("imu_pub_rate", imu_pub_rate, 250.0);
  nh_private_.param<double>("rc_send_rate", rc_send_rate, 100.0);
  nh_private_.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  nh_private_.param<int>("baudrate", baudrate, 115200);
  nh_private_.param<double>("timeout", dtimeout, 2);


  // connect serial port
  serial::Timeout timeout = serial::Timeout::simpleTimeout(dtimeout);
  MSP_ = new MSP(serial_port, (u_int32_t)baudrate, timeout);

  // Set up Callbacks
  Command_subscriber_ = nh_.subscribe("command", 1, &nazeROS::RPYCallback, this);
  RC_calibration_subscriber_ = nh_.subscribe("calibrate", 1, &nazeROS::calibrationCallback, this);
  arm_subscriber_ = nh_.subscribe("arm", 1, &nazeROS::armCallback, this);
  imu_pub_timer_ = nh_.createTimer(ros::Duration(1.0/imu_pub_rate), &nazeROS::imuCallback, this);
  rc_send_timer_ = nh_.createTimer(ros::Duration(1.0/rc_send_rate), &nazeROS::rcCallback, this);

  // setup publishers
  Imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);

  // initialize RC variables
  center_sticks_.resize(8);
  max_sticks_.resize(8);
  min_sticks_.resize(8);
  rc_commands_.resize(8);
  ROS_WARN("RC Calibration Settings");
  loadRCFromParam();
  armed_ = false;
  acro_ = false;

  //initialize constant message members
  Imu_.header.frame_id = imu_frame_id;
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

//  if(calibrateIMU()){
//    ROS_INFO("IMU calibration successful");
//  }else{
//    ROS_ERROR("IMU calibration unsuccessful");
//  }
  ROS_INFO("finished initialization");
}

nazeROS::~nazeROS(){
  delete MSP_;
}


void nazeROS::calibrationCallback(const std_msgs::BoolConstPtr &msg)
{
  if(msg->data){
    if(calibrateRC()){
      ROS_INFO("RC Calibration Successful");
    }
  }
}

void nazeROS::armCallback(const std_msgs::BoolConstPtr &msg)
{
  armed_ = msg->data;
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
      PWM_range = (max_sticks_[i] - min_sticks_[i])/2;
      rc_commands_[i] = (uint16_t)sat((int)round(command[i]*PWM_range+center_sticks_[i]),min_PWM_output_, max_PWM_output_);
    }else if(i == RC_THR){
      PWM_range = max_sticks_[i] - min_sticks_[i];
      rc_commands_[i] = (uint16_t)sat((int)round(command[i]*PWM_range+min_sticks_[i]),min_PWM_output_, max_PWM_output_);
    }else if(i == RC_ARM){
      rc_commands_[RC_ARM] = armed_?max_PWM_output_:min_PWM_output_;
    }else if(i == RC_ACRO){
      rc_commands_[RC_ACRO] = acro_?max_PWM_output_:min_PWM_output_;
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
  for(int i=0; i<200; i++){
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
    center_sticks_[i] = (uint16_t)round(sum[i]/200.0);
    ROS_INFO_STREAM("Channel " << i << " trim = " << center_sticks_[i]);
  }

  ROS_WARN("Calibrating maximum and minimum of RC channels");
  ROS_WARN("Provide Full Throttle Command with Joystick and");
  ROS_WARN("move RC sticks to full extent");
  uint16_t max_PWM_received[4] = {center_sticks_[0], center_sticks_[1], center_sticks_[2], center_sticks_[3]};
  uint16_t min_PWM_received[4] = {center_sticks_[0], center_sticks_[1], center_sticks_[2], center_sticks_[3]};
  for(int i=0; i<200; i++){
    success = MSP_->getRC(read_rc_commands);
    if(success){
      for(int j=0; j<4; j++){
          max_PWM_received[j] = (uint16_t)(max_PWM_received[j] > read_rc_commands.rcData[j]) ? max_PWM_received[j] : read_rc_commands.rcData[j];
          min_PWM_received[j] = (uint16_t)(min_PWM_received[j] < read_rc_commands.rcData[j]) ? min_PWM_received[j] : read_rc_commands.rcData[j];
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

bool nazeROS::sendRC()
{
  SetRawRC outgoing_rc_commands;
  for(int i=0; i<8; i++){
    outgoing_rc_commands.rcData[i] = rc_commands_[i];
  }
  return MSP_->setRawRC(outgoing_rc_commands);
//  return getRC();
}

bool nazeROS::getRC()
{
  RC actual_rc_commands;
  bool success = MSP_->getRC(actual_rc_commands);
  ROS_INFO_STREAM("RC Commands:");
  for(int i=0; i<8; i++){
    ROS_INFO_STREAM(i <<": = " << actual_rc_commands.rcData[i]);
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

bool nazeROS::loadRCFromParam()
{
  int max_ail, max_ele, max_rud, max_thr;
  int min_ail, min_ele, min_rud, min_thr;
  int mid_ail, mid_ele, mid_rud, mid_thr;
  nh_private_.param<int>("roll/max",  max_ail, 2000);
  nh_private_.param<int>("pitch/max", max_ele, 2000);
  nh_private_.param<int>("yaw/max",   max_rud, 2000);
  nh_private_.param<int>("thrust/max",max_thr, 2000);

  nh_private_.param<int>("roll/min",   min_ail, 1000);
  nh_private_.param<int>("pitch/min",  min_ele, 1000);
  nh_private_.param<int>("yaw/min",    min_rud, 1000);
  nh_private_.param<int>("thrust/min", min_thr, 1000);

  nh_private_.param<int>("roll/center",   mid_ail, 1500);
  nh_private_.param<int>("pitch/center",  mid_ele, 1500);
  nh_private_.param<int>("yaw/center",    mid_rud, 1500);
  nh_private_.param<int>("thrust/center", mid_thr, 1500);

  max_sticks_[RC_AIL] = max_ail;
  max_sticks_[RC_ELE] = max_ele;
  max_sticks_[RC_RUD] = max_rud;
  max_sticks_[RC_THR] = max_thr;
  min_sticks_[RC_AIL] = min_ail;
  min_sticks_[RC_ELE] = min_ele;
  min_sticks_[RC_RUD] = min_rud;
  min_sticks_[RC_THR] = min_thr;
  center_sticks_[RC_AIL] = mid_ail;
  center_sticks_[RC_ELE] = mid_ele;
  center_sticks_[RC_RUD] = mid_rud;
  center_sticks_[RC_THR] = mid_thr;

  for(int i = 0; i<4; i++){
    ROS_WARN_STREAM("max = " << max_sticks_[i] << " min = " << min_sticks_[i] << " center = " << center_sticks_[i]);
    rc_commands_[i] = (i == RC_THR)?min_PWM_output_:center_sticks_[i];
  }
  for(int i=4; i<8; i++){
    rc_commands_[i] = min_PWM_output_;
  }
  return true;
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
