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

  bool getStatus(Status& message);
  bool getRawIMU(RawIMU& message);
  bool calibrateIMU();
  bool getAttitude(Attitude& message);
  bool setRawRC(SetRawRC& message);
  bool getRC(RC& message);
  bool getPID(PID& message);
  bool setPID(SetPID& message);
  bool getRawAirspeed(Airspeed& message);
  bool getAltitude(Altitude& message);
  bool getSonar(Sonar& message);


private:
  serial::Serial Serial_;

  // template-specific functions - these can be expanded for additional messages
  bool receive(RawIMU& message);
  bool send(SetRawRC& command);
  bool receive(RC& message);
  bool receive(Attitude& message);
  bool send(SetPID& command);
  bool receive(PID& message);
  bool receive(Status& message);
  bool receive(Airspeed& message);
  bool receive(Altitude& message);
  bool receive(Sonar& message);

  // unspecific communication functions
  bool acknowledge(u_int8_t code);
  bool request(u_int8_t code);
  bool send(u_int8_t code, u_int8_t* data, u_int8_t size);
  bool receive(u_int8_t code, u_int8_t size, u_int8_t* data);

  // helper functions for specific tasks
  u_int8_t calculate_checksum(u_int8_t* data, u_int8_t size, u_int8_t init);
};

#endif // MSP_H
