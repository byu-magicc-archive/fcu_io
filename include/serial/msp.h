#ifndef MSP_H
#define MSP_H

#include <serial/serial.h>
#include <serial/mspdata.h>
#include <bitset>
#include <unistd.h>
#include <iostream>

typedef unsigned char u_int8_t;
typedef unsigned short u_int16_t;

class MSP
{
public:
  MSP(std::string port, uint32_t baud, serial::Timeout timeout);
  bool getRawIMU(RawIMU& message);

  bool request(u_int8_t code);
  bool sendAttitude(Attitude command);
  bool receive(Attitude& message);
  bool receive(RawIMU& message);
  template<class T> bool acknowledge();

private:
  u_int8_t calculate_checksum(u_int8_t* data, u_int8_t size, u_int8_t init);
  serial::Serial Serial_;
  bool send(u_int8_t code, u_int8_t* data, u_int8_t size);
  bool receive(u_int8_t code, u_int8_t size, u_int8_t* data);
};

#endif // MSP_H
