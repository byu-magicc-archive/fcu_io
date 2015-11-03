#include "serial/msp.h"
#include <ros/ros.h>

MSP::MSP(std::string port, uint32_t baud, serial::Timeout timeout)
  : Serial_(port, baud, timeout)
{
}

bool MSP::getRawIMU(RawIMU &message)
{
  if(request(RawIMU::type))
  {
    return receive(message);
  }
}

bool MSP::request(u_int8_t code)
{
  return send(code, 0, 0);
}


bool MSP::receive(u_int8_t code, u_int8_t size, u_int8_t* data)
{
  // read data
  std::vector<u_int8_t> message_buffer;
  size_t header_length = Serial_.read(message_buffer, (u_int8_t)(size+6));

  // check header
  if(message_buffer[0] != (u_int8_t)'$' ||
     message_buffer[1] != (u_int8_t)'M' ||
     message_buffer[2] != (u_int8_t)'>'){
    ROS_ERROR_STREAM("recieved incorrect signature, " << message_buffer[0] << message_buffer[1] << message_buffer[2]);
    return false;
  }else if(message_buffer[3] != size){
    ROS_ERROR_STREAM("Received incorrect size");
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


bool MSP::sendAttitude(Attitude command)
{
  u_int16_t data[3] = {command.angx, command.angy, command.heading};
  return send(command.type, reinterpret_cast<u_int8_t*>(&data), sizeof(data));
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


