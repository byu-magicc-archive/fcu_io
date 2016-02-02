#include "fcu_io/fcu_io.h"

namespace fcu_io
{

fcuIO::fcuIO() :
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
  nh_private_.param<double>("rc_send_rate", rc_send_rate, 50.0);
  nh_private_.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
  nh_private_.param<int>("baudrate", baudrate, 115200);
  nh_private_.param<double>("timeout", dtimeout, 2);

  // connect serial port
  serial::Timeout timeout = serial::Timeout::simpleTimeout(dtimeout);
  timeout.inter_byte_timeout = serial::Timeout::max();
  timeout.read_timeout_constant = 1;
  timeout.write_timeout_constant = 2;
  MSP_ = new MSP(serial_port, (u_int32_t)baudrate, timeout);

  // Set up Callbacks
  Command_subscriber_ = nh_.subscribe("command", 1, &fcuIO::RPYCallback, this);
  RC_calibration_subscriber_ = nh_.subscribe("calibrate", 1, &fcuIO::calibrationCallback, this);
//  arm_subscriber_ = nh_.subscribe("arm", 1, &fcuIO::armCallback, this);
  imu_pub_timer_ = nh_.createTimer(ros::Duration(1.0/imu_pub_rate), &fcuIO::imuCallback, this);
  rc_send_timer_ = nh_.createTimer(ros::Duration(1.0/rc_send_rate), &fcuIO::rcCallback, this);
  as_pub_timer_ = nh_.createTimer(ros::Duration(0.05), &fcuIO::asCallback, this);
  alt_pub_timer_ = nh_.createTimer(ros::Duration(0.05), &fcuIO::altCallback, this);
  sonar_pub_timer_ = nh_.createTimer(ros::Duration(0.05), &fcuIO::sonarCallback, this);

  // setup publishers
  Imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
  Mag_publisher_ = nh_.advertise<sensor_msgs::MagneticField>("mag/data", 1);
  Baro_alt_publisher_ = nh_.advertise<std_msgs::Float32>("baro/alt",1);
  Airspeed_publisher_ = nh_.advertise<sensor_msgs::FluidPressure>("airspeed/data", 1);
  Sonar_publisher_ = nh_.advertise<sensor_msgs::Range>("sonar/data", 1);

  // initialize RC variables
  center_sticks_.resize(8);
  max_sticks_.resize(8);
  min_sticks_.resize(8);
  rc_commands_.resize(8);
  PIDs_.resize(10);
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
  getPID();

  // dynamic reconfigure
  func_ = boost::bind(&fcuIO::gainCallback, this, _1, _2);
  server_.setCallback(func_);

  ROS_INFO("finished initialization");
}

fcuIO::~fcuIO(){
  delete MSP_;
}


void fcuIO::calibrationCallback(const std_msgs::BoolConstPtr &msg)
{
  if(msg->data){
    if(calibrateRC()){
      ROS_INFO("RC Calibration Successful");
    }
  }
}

void fcuIO::armCallback(const std_msgs::BoolConstPtr &msg)
{
  armed_ = msg->data;
}

void fcuIO::RPYCallback(const fcu_io::CommandConstPtr &msg)
{
  int aux1(0.0), aux2(0.0), aux3(0.0), aux4(0.0);
  double command[4] = {0.0, 0.0, 0.0, 0.0};
  uint16_t PWM_range;
  command[RC_AIL] = msg->normalized_roll/max_roll_;
  command[RC_ELE] = msg->normalized_pitch/max_pitch_;
  command[RC_THR] = msg->normalized_throttle/max_throttle_;
  command[RC_RUD] = msg->normalized_yaw/max_yaw_rate_;

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

void fcuIO::imuCallback(const ros::TimerEvent& event)
{
  if(!getImu()){
    ROS_ERROR_STREAM("IMU receive error");
  }
}


void fcuIO::rcCallback(const ros::TimerEvent& event)
{
  if(!sendRC()){
    ROS_ERROR_STREAM("RC Send Error");
  }
}

void fcuIO::asCallback(const ros::TimerEvent& event)
{
  Airspeed receivedASdata;
  memset(&receivedASdata, 0, sizeof(receivedASdata));
  bool received = MSP_->getRawAirspeed(receivedASdata);
  static int calibration_counter = 0;
  const int calibration_count = 100;
  static float _diff_pres_offset = 0;

  if(received){
      if(receivedASdata.airspeed != 0)
      {
          // conversion from pixhawk source code
          float temperature = ((200.0f * receivedASdata.temp) / 2047) - 50;
          const float P_min = -1.0f;
          const float P_max = 1.0f;
          const float PSI_to_Pa = 6894.757f;
          /*
            this equation is an inversion of the equation in the
            pressure transfer function figure on page 4 of the datasheet

            We negate the result so that positive differential pressures
            are generated when the bottom port is used as the static
            port on the pitot and top port is used as the dynamic port
           */
          float diff_press_PSI = -((receivedASdata.airspeed - 0.1f*16383) * (P_max-P_min)/(0.8f*16383) + P_min);
          float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;
          if(calibration_counter > calibration_count)
          {
              diff_press_pa_raw -= _diff_pres_offset;
              sensor_msgs::FluidPressure pressure_msg;
              pressure_msg.fluid_pressure = diff_press_pa_raw;
              Airspeed_publisher_.publish(pressure_msg);
              //ROS_INFO_STREAM("AIRSPEED = " << sqrt(2/1.15*diff_press_pa_raw) << " pressure " << diff_press_pa_raw);
          }
          else if(calibration_counter == calibration_count)
          {
              _diff_pres_offset = _diff_pres_offset/calibration_count;
              calibration_counter++;
          }
          else
          {
              _diff_pres_offset += diff_press_pa_raw;
              calibration_counter++;
          }
      }
  } else{
        ROS_ERROR_STREAM("Airspeed receive error");
  }
}

void fcuIO::altCallback(const ros::TimerEvent& event)
{
  Altitude receivedAltdata;
  memset(&receivedAltdata, 0, sizeof(receivedAltdata));
  bool received = MSP_->getAltitude(receivedAltdata);

  if(received){
      std_msgs::Float32 alt;
      alt.data = receivedAltdata.estAlt/100.0;
      Baro_alt_publisher_.publish(alt);
//      ROS_INFO_STREAM("ALT = " << receivedAltdata.estAlt << " VARIO = " << receivedAltdata.vario);
  } else{
        ROS_ERROR_STREAM("ALT receive error");
  }
}

void fcuIO::sonarCallback(const ros::TimerEvent& event)
{
  Sonar receivedSdata;
  memset(&receivedSdata, 0, sizeof(receivedSdata));
  bool received = MSP_->getSonar(receivedSdata);

  if(received && receivedSdata.distance > 2 && receivedSdata.distance < 400){
      sensor_msgs::Range dist;
      //dist.header.stamp.now();
      dist.min_range = 0.02;
      dist.max_range = 4;
      dist.radiation_type = dist.ULTRASOUND;
      dist.field_of_view = 15*M_PI/180.0;
      dist.range = receivedSdata.distance/100.0;
      Sonar_publisher_.publish(dist);
//      ROS_INFO_STREAM("SONAR = " << receivedSdata.distance);
  } else{
        ROS_ERROR_STREAM("Sonar receive error");
  }
}

bool fcuIO::calibrateIMU()
{
  MSP_->calibrateIMU();
}


bool fcuIO::calibrateRC()
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

bool fcuIO::sendRC()
{
  SetRawRC outgoing_rc_commands;
  for(int i=0; i<8; i++){
    outgoing_rc_commands.rcData[i] = rc_commands_[i];
  }
  MSP_->setRawRC(outgoing_rc_commands);
  //return getRC();
}

bool fcuIO::setPID(PIDitem roll, PIDitem pitch, PIDitem yaw)
{
  SetPID outgoing_PID_command;
  for(int i = 0; i<10; i++){
    if(i == ROLL){
      outgoing_PID_command.PIDs[ROLL].P = (uint8_t)(roll.P*10); // convert back to chars for MSP
      outgoing_PID_command.PIDs[ROLL].I = (uint8_t)(roll.I*1000);
      outgoing_PID_command.PIDs[ROLL].D = (uint8_t)(roll.D*1);
    }else if( i == PITCH){
      outgoing_PID_command.PIDs[PITCH].P = (uint8_t)(pitch.P*10);
      outgoing_PID_command.PIDs[PITCH].I = (uint8_t)(pitch.I*1000);
      outgoing_PID_command.PIDs[PITCH].D = (uint8_t)(pitch.D*1);
    }else if( i == YAW) {
      outgoing_PID_command.PIDs[YAW].P = (uint8_t)(yaw.P*10);
      outgoing_PID_command.PIDs[YAW].I = (uint8_t)(yaw.I*1000);
      outgoing_PID_command.PIDs[YAW].D = (uint8_t)(yaw.D*1);
    }else{
      outgoing_PID_command.PIDs[i].P = (uint8_t)(PIDs_[i].P*10);
      outgoing_PID_command.PIDs[i].I = (uint8_t)(PIDs_[i].I*1000);
      outgoing_PID_command.PIDs[i].D = (uint8_t)(PIDs_[i].D*1);
    }
  }
  MSP_->setPID(outgoing_PID_command);
  return getPID();
}

bool fcuIO::getRC()
{
  RC actual_rc_commands;
  bool success = MSP_->getRC(actual_rc_commands);
  ROS_INFO_STREAM("RC Commands:");
  for(int i=0; i<8; i++){
    ROS_INFO_STREAM(i <<": = " << actual_rc_commands.rcData[i]);
  }
  return true;
}


bool fcuIO::getImu()
{
  RawIMU receivedIMUdata;
  memset(&receivedIMUdata, 0, sizeof(receivedIMUdata));
  bool received = MSP_->getRawIMU(receivedIMUdata);
//  RC receivedRCdata;
//  bool received = MSP_->getRC(receivedRCdata);
  if(received){
    Imu_.linear_acceleration.x = (double)receivedIMUdata.accx/512.0*9.80665;
    Imu_.linear_acceleration.y = -1.0*(double)receivedIMUdata.accy/512.0*9.80665;
    Imu_.linear_acceleration.z = -1.0*(double)receivedIMUdata.accz/512.0*9.80665;
    Imu_.angular_velocity.x = (double)receivedIMUdata.gyrx*0.001065264; // 2^15 = 2000 deg/s
    Imu_.angular_velocity.y = -1.0*(double)receivedIMUdata.gyry*0.001065264;
    Imu_.angular_velocity.z = -1.0*(double)receivedIMUdata.gyrz*0.001065264;
    Imu_.header.stamp = ros::Time::now();
    Imu_publisher_.publish(Imu_);

    sensor_msgs::MagneticField mag;
    mag.magnetic_field.x = receivedIMUdata.magx;
    mag.magnetic_field.y = receivedIMUdata.magy;
    mag.magnetic_field.z = receivedIMUdata.magz;
    Mag_publisher_.publish(mag);
    //ROS_INFO_STREAM(receivedIMUdata.magx << " " << receivedIMUdata.magy << " " << receivedIMUdata.magz);

    return true;
  } else{
    return false;
  }
}

bool fcuIO::getAS()
{
  Airspeed receivedASdata;
  memset(&receivedASdata, 0, sizeof(receivedASdata));
  bool received = MSP_->getRawAirspeed(receivedASdata);

  if(received){
    //if(receivedASdata.airspeed != 0)
        ROS_INFO_STREAM("AIRSPEED = " << receivedASdata.airspeed << " TEMP = " << receivedASdata.temp);
    return true;
  } else{
        ROS_INFO_STREAM("nothing");
    return false;
  }
}

bool fcuIO::getPID(){
  PID receivedPIDmsg;
  memset(&receivedPIDmsg, 0, sizeof(receivedPIDmsg));
  bool received = MSP_->getPID(receivedPIDmsg);
  if(received){
    for(int i = 0; i<10; i++){
      PIDs_[i].P = (double)receivedPIDmsg.PIDs[i].P/10.0; // convert back to double
      PIDs_[i].I = (double)receivedPIDmsg.PIDs[i].I/1000.0; // convert back to double
      PIDs_[i].D = (double)receivedPIDmsg.PIDs[i].D/1.0; // convert back to double
    }
    ROS_INFO_STREAM("Gains received: Roll = " <<  PIDs_[ROLL].P << ", "  << PIDs_[ROLL].I << ", " <<  PIDs_[ROLL].D);
    ROS_INFO_STREAM("Gains received: PITCH = " <<  PIDs_[PITCH].P << ", "  << PIDs_[PITCH].I << ", " <<  PIDs_[PITCH].D);
    ROS_INFO_STREAM("Gains received: YAW = " <<  PIDs_[YAW].P << ", "  << PIDs_[YAW].I << ", " <<  PIDs_[YAW].D);
    return true;
  } else {
    return false;
  }
}

bool fcuIO::loadRCFromParam()
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

void fcuIO::gainCallback(fcu_io::GainConfig &config, uint32_t level)
{
  PIDitem new_roll, new_pitch, new_yaw;
  new_roll.P = config.rollP;
  new_roll.I = config.rollI;
  new_roll.D = config.rollD;
  new_pitch.P = config.pitchP;
  new_pitch.I = config.pitchI;
  new_pitch.D = config.pitchD;
  new_yaw.P = config.yawP;
  new_yaw.I = config.yawI;
  new_yaw.D = config.yawD;
  setPID(new_roll, new_pitch, new_yaw);
  ROS_INFO("new gains");
  return;
}


int fcuIO::sat(int input, int min, int max){
  int output(input);
  if(input > max){
    output = max;
  }else if(input < min){
    output = min;
  }
  return output;
}



} // namespace fcu_io
