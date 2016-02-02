#include "serial/msp.h"
#include <ros/ros.h>

MSP::MSP(std::string port, uint32_t baud, serial::Timeout timeout)
  : Serial_(port, baud, timeout)
{
}

bool MSP::getPID(PID& message)
{
  if(request(PID::type)){
    return receive(message);
  }else{
    return false;
  }
}

bool MSP::setPID(SetPID& message)
{
  if(!send(message)){
    return false;
  }else{
    return acknowledge(SetPID::type);
  }
}

bool MSP::getStatus(Status &message)
{
  if(request(Status::type)){
    return receive(message);
  }else{
    return false;
  }
}

bool MSP::getRawIMU(RawIMU &message)
{
  if(request(RawIMU::type)){
    return receive(message);
  }else{
    return false;
  }
}
bool MSP::getRawAirspeed(Airspeed &message)
{
  if(request(Airspeed::type)){
    return receive(message);
  }else{
    return false;
  }
}

bool MSP::getAttitude(Attitude& message)
{
  if(request(Attitude::type)){
    return receive(message);
  }else{
    return false;
  }
}

bool MSP::getAltitude(Altitude& message)
{
  if(request(Altitude::type)){
    return receive(message);
  }else{
    return false;
  }
}

bool MSP::getSonar(Sonar& message)
{
  if(request(Sonar::type)){
    return receive(message);
  }else{
    return false;
  }
}

bool MSP::getRC(RC &message)
{
  if(request(RC::type)){
    return receive(message);
  }else{
    return false;
  }
}


bool MSP::setRawRC(SetRawRC& message)
{
  if(!send(message)){
    return false;
  }else{
    return acknowledge(SetRawRC::type);
  }
}



bool MSP::request(u_int8_t code)
{
  Serial_.flush();
  return send(code, 0, 0);
}


bool MSP::calibrateIMU()
{
  if(send(AccCalibration::type, 0, 0)){
    sleep(3);
    return acknowledge(AccCalibration::type);
  }
}

bool MSP::receive(PID &message)
{
  uint8_t size = sizeof(PID);
  uint8_t code = (uint8_t)PID::type;
  uint8_t data[size];
  if(receive(code, size, data)) {
    for(int i = 0; i<10; i++){
      message.PIDs[i].P = data[3*i];
      message.PIDs[i].I = data[3*i+1];
      message.PIDs[i].D = data[3*i+2];
    }
  }
}

bool MSP::receive(Status& message)
{
  u_int8_t size = sizeof(Status);
  u_int8_t code = (u_int8_t)Status::type;
  u_int8_t data[size];
  if(receive(code, size, data))  {
    message.cycle_time = (int16_t)(((data[1] & 0xFF) << 8) | (data[0] & 0xFF));
    message.i2c_errors_count = (int16_t)(((data[3] & 0xFF) << 8) | (data[2] & 0xFF));
    message.sensor = (int16_t)(((data[5] & 0xFF) << 8) | (data[4] & 0xFF));
    message.flag = (int32_t)(((data[9] & 0xFF) << 24) | ((data[8] & 0xFF) << 16) | ((data[7] & 0xFF) << 8) | (data[6] & 0xFF));
    message.current_set = (int16_t)(((data[11] & 0xFF) << 8) | (data[10] & 0xFF));
    return true;
  } else {
    return false;
  }
}

bool MSP::receive(RawIMU& message)
{
  u_int8_t size = sizeof(RawIMU);
  u_int8_t code = (u_int8_t)RawIMU::type;
  u_int8_t data[size];
  if(receive(code, size, data))  {
    message.accx = (int16_t)(((data[1] & 0xFF) << 8) | (data[0] & 0xFF));
    message.accy = (int16_t)(((data[3] & 0xFF) << 8) | (data[2] & 0xFF));
    message.accz = (int16_t)(((data[5] & 0xFF) << 8) | (data[4] & 0xFF));
    message.gyrx = (int16_t)(((data[7] & 0xFF) << 8) | (data[6] & 0xFF));
    message.gyry = (int16_t)(((data[9] & 0xFF) << 8) | (data[8] & 0xFF));
    message.gyrz = (int16_t)(((data[11] & 0xFF) << 8) | (data[10] & 0xFF));
    message.magx = (int16_t)(((data[13] & 0xFF) << 8) | (data[12] & 0xFF));
    message.magy = (int16_t)(((data[15] & 0xFF) << 8) | (data[14] & 0xFF));
    message.magz = (int16_t)(((data[17] & 0xFF) << 8) | (data[16] & 0xFF));
    return true;
  } else {
    return false;
  }
}

bool MSP::receive(Attitude& message)
{
  u_int8_t size = sizeof(Attitude);
  u_int8_t code = (u_int8_t)Attitude::type;
  u_int8_t data[size];
  if(receive(code, size, data))
  {
    message.angx = (int16_t)(((data[1] & 0xFF) << 8) | (data[0] & 0xFF));
    message.angy = (int16_t)(((data[3] & 0xFF) << 8) | (data[2] & 0xFF));
    message.heading = (int16_t)(((data[5] & 0xFF) << 8) | (data[4] & 0xFF));
  }
}

bool MSP::receive(Airspeed& message)
{
  u_int8_t size = sizeof(Airspeed);
  u_int8_t code = (u_int8_t)Airspeed::type;
  u_int8_t data[size];
  if(receive(code, size, data))
  {
    message.airspeed = (int16_t)(((data[1] & 0xFF) << 8) | (data[0] & 0xFF));
    message.temp = (int16_t)(((data[3] & 0xFF) << 8) | (data[2] & 0xFF));
  }
}

bool MSP::receive(Altitude& message)
{
  u_int8_t size = sizeof(Altitude);
  u_int8_t code = (u_int8_t)Altitude::type;
  u_int8_t data[size];
  if(receive(code, size, data))
  {
    message.estAlt = (int32_t)(((data[3] & 0xFF) << 24) | ((data[2] & 0xFF) << 16) | ((data[1] & 0xFF) << 8) | (data[0] & 0xFF));
    message.vario = (int16_t)(((data[5] & 0xFF) << 8) | (data[4] & 0xFF));
  }
}

bool MSP::receive(Sonar& message)
{
  u_int8_t size = sizeof(Sonar);
  u_int8_t code = (u_int8_t)Sonar::type;
  u_int8_t data[size];
  if(receive(code, size, data))
  {
    message.distance = (int32_t)(((data[3] & 0xFF) << 24) | ((data[2] & 0xFF) << 16) | ((data[1] & 0xFF) << 8) | (data[0] & 0xFF));
  }
}

bool MSP::receive(RC &message)
{
  u_int8_t size = sizeof(RC);
  u_int8_t code = (u_int8_t)RC::type;
  u_int8_t data[size];
  if(receive(code, size, data))
  {
    for(int i=0; i<8; i++){
      message.rcData[i] = (int16_t)(((data[2*i+1] & 0xFF) << 8) | (data[2*i] & 0xFF));
    }
  }
}

bool MSP::send(SetPID &command)
{
  uint8_t size = sizeof(SetPID);
  uint8_t data[size];
  for(int i=0; i<10; i++){
    data[3*i] = command.PIDs[i].P;
    data[3*i+1] = command.PIDs[i].I;
    data[3*i+2] = command.PIDs[i].D;
  }
  return send(command.type, reinterpret_cast<uint8_t*>(&data), sizeof(data));
}


bool MSP::send(SetRawRC &command)
{
  u_int16_t data[8];
  for(int i=0; i<8; i++){
     data[i] = command.rcData[i];
  }
  return send(command.type, reinterpret_cast<u_int8_t*>(&data), sizeof(data));
}


bool MSP::acknowledge(u_int8_t code)
{
  u_int8_t* data;
  return receive(code, 0, data);
}



bool MSP::receive(u_int8_t code, u_int8_t size, u_int8_t* data)
{
  // read data
  std::vector<u_int8_t> message_buffer;
  while((int)Serial_.available() < (int)size+6) {}
  if((int)Serial_.available() > (int)size+6){
    ROS_ERROR_STREAM((int)Serial_.available() - (int)size -6 << "extra bits on serial port");
  }
  size_t header_length = Serial_.read(message_buffer, (u_int8_t)(size+6));

  // DEBUG
//  if(error){
//    ROS_INFO_STREAM("recieved ");
//    for(int i = 0; i<(int)size+6; i++){
//      std::bitset<8> output(message_buffer[i]);
//      ROS_INFO_STREAM(message_buffer[i] << ": " <<output);
//    }
//  }

  // check header
  if(message_buffer[0] != (u_int8_t)'$' ||
     message_buffer[1] != (u_int8_t)'M' ||
     message_buffer[2] != (u_int8_t)'>'){
    ROS_ERROR_STREAM("received incorrect signature, ");
    for(int i=0; i<3; i++){
      std::bitset<8> byte(message_buffer[i]);
     ROS_ERROR_STREAM( byte << ": " << message_buffer[i]);
    }
    return false;
  }else if(message_buffer[3] != size){
    ROS_ERROR_STREAM("Received incorrect size" << (int)message_buffer[3] << " vs. " << (int)size);
    return false;
  }else if(message_buffer[4] != code){
    ROS_ERROR_STREAM("Received incorrect code");
    return false;
  }

  // pull in data
  for(int k=0; k<(int)size; k++){
    data[k] = message_buffer[k+5];
  }

  // calcualate checksum
  u_int8_t checksum = calculate_checksum(data, size, code^size);
  if(checksum != message_buffer[message_buffer.size()-1]){
    ROS_ERROR_STREAM("Serial Error, Incorrect Checksum");
    return false;
  }else{
    return true;
  }
}


bool MSP::send(u_int8_t code, u_int8_t* data, u_int8_t size)
{
  // compile MSP header
  std::vector<u_int8_t> output_buffer(6+(int)size);
  output_buffer[0] = (u_int8_t)'$';
  output_buffer[1] = (u_int8_t)'M';
  output_buffer[2] = (u_int8_t)'<';
  output_buffer[3] = size;
  output_buffer[4] = code;

  // compile data
  for(int j=0; j<(int)size; j++){
    output_buffer[j+5] = data[j];
  }

  // calcualate checksum
  u_int8_t checksum_init = code^size;
  output_buffer[5+(int)size] = calculate_checksum(data, size, checksum_init);

  // send message
  size_t bytes_wrote = Serial_.write(output_buffer);


  // DEBUG
//  ROS_INFO_STREAM("sent ");
//  for(int i=0; i<(int)size+6; i++){
//    std::bitset<8> output(output_buffer[i]);
//    ROS_INFO_STREAM( output_buffer[i] << ": " << output);
//  }

  // return code
  return bytes_wrote == output_buffer.size();
}


u_int8_t MSP::calculate_checksum(u_int8_t* data, u_int8_t size, u_int8_t init){
  u_int8_t checksum = init;
  for(int i=0; i<(int)size; i++){
    checksum = checksum^data[i];
  }
  return checksum;
}


