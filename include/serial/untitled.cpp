#pragma once
#include <stdint.h>
#include <iosfwd>

typedef unsigned char u_int8_t;
typedef unsigned short u_int16_t;

u_int8_t MSP_IDENT = 100;
u_int8_t MSP_STATUS = 101;
u_int8_t MSP_RAW_IMU = 102;
u_int8_t MSP_SERVO = 103;
u_int8_t MSP_MOTOR = 104;
u_int8_t MSP_SET_MOTOR = 214;
u_int8_t MSP_RC = 105;
u_int8_t MSP_SET_RAW_RC = 200;
u_int8_t MSP_RAW_GPS = 106;
u_int8_t MSP_SET_RAW_GPS = 201;
u_int8_t MSP_COMP_GPS = 107;
u_int8_t MSP_ATTITUDE = 108;
u_int8_t MSP_ALTITUDE = 109;
u_int8_t MSP_ANALOG = 110;
u_int8_t MSP_RC_TUNING = 111;
u_int8_t MSP_SET_RC_TUNING = 204;
u_int8_t MSP_PID = 112;
u_int8_t MSP_SET_PID = 202;
u_int8_t MSP_BOX = 113;
u_int8_t MSP_SET_BOX = 203;
u_int8_t MSP_MISC = 114;
u_int8_t MSP_SET_MISC = 207;
u_int8_t MSP_MOTOR_PINS = 115;
u_int8_t MSP_BOXNAMES = 116;
u_int8_t MSP_PIDNAMES = 117;
u_int8_t MSP_WP = 118;
u_int8_t MSP_SET_WP = 209;
u_int8_t MSP_BOXIDS = 119;
u_int8_t MSP_SERVO_CONF = 120;
u_int8_t MSP_SET_SERVO_CONF = 212;
u_int8_t MSP_ACC_CALIBRATION = 205;
u_int8_t MSP_MAG_CALIBRATION = 206;
u_int8_t MSP_RESET_CONF = 208;
u_int8_t MSP_SELECT_SETTING = 210;
u_int8_t MSP_SET_HEAD = 211;
u_int8_t MSP_BIND = 240;
u_int8_t MSP_EEPROM_WRITE = 250;

struct Ident
{
  const static u_int8_t type = 100;
  u_int8_t version;
  u_int8_t multi_type;
  u_int8_t msp_version;
  u_int32_t capability;
} __attribute__((packed));

struct Status
{
  const static u_int8_t type = 101;
  u_int16_t cycle_time;
  u_int16_t i2c_errors_count;
  u_int16_t sensor;
  uint32_t flag;
  u_int8_t current_set;
} __attribute__((packed));

struct RawIMU
{
  const static u_int8_t type = 102;
  int16_t accx;
  int16_t accy;
  int16_t accz;
  int16_t gyrx;
  int16_t gyry;
  int16_t gyrz;
  int16_t magx;
  int16_t magy;
  int16_t magz;
} __attribute__((packed));

struct Servo
{
  const static u_int8_t type = 103;
  u_int16_t servo[8];
} __attribute__((packed));

struct Motor
{
  const static u_int8_t type = 104;
  u_int16_t motor[8];
} __attribute__((packed));

struct RC
{
  const static u_int8_t type = 105;
  u_int16_t rcData[8];
} __attribute__((packed));

struct SetRawRC
{
  const static u_int8_t type = 200;
  u_int16_t rcData[8];
} __attribute__((packed));

struct Attitude
{
  const static u_int8_t type = 108;
  int16_t angx;
  int16_t angy;
  int16_t heading;
} __attribute__((packed));

struct Altitude
{
  const static u_int8_t type = 109;
  int32_t estAlt;
  int16_t vario;
} __attribute__((packed));

struct AccCalibration
{
  const static u_int8_t type = 205;
} __attribute__((packed));
