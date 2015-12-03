#pragma once
#include <stdint.h>
#include <iosfwd>

// PID channels
#define ROLL 0
#define PITCH 1
#define YAW 2
#define ALT 3
#define POS 4
#define POSR 5
#define NAVR 6
#define LEVEL 7
#define MAG 8
#define VEL 9

enum class Command : uint8_t
{
  MSP_IDENT = 100,
  MSP_STATUS = 101,
  MSP_RAW_IMU = 102,
  MSP_SERVO = 103,
  MSP_MOTOR = 104,
  MSP_SET_MOTOR = 214,
  MSP_RC = 105,
  MSP_SET_RAW_RC = 200,
  MSP_RAW_GPS = 106,
  MSP_SET_RAW_GPS = 201,
  MSP_COMP_GPS = 107,
  MSP_ATTITUDE = 108,
  MSP_ALTITUDE = 109,
  MSP_ANALOG = 110,
  MSP_RC_TUNING = 111,
  MSP_SET_RC_TUNING = 204,
  MSP_PID = 112,
  MSP_SET_PID = 202,
  MSP_BOX = 113,
  MSP_SET_BOX = 203,
  MSP_MISC = 114,
  MSP_SET_MISC = 207,
  MSP_MOTOR_PINS = 115,
  MSP_BOXNAMES = 116,
  MSP_PIDNAMES = 117,
  MSP_WP = 118,
  MSP_SET_WP = 209,
  MSP_BOXIDS = 119,
  MSP_SERVO_CONF = 120,
  MSP_SET_SERVO_CONF = 212,
  MSP_ACC_CALIBRATION = 205,
  MSP_MAG_CALIBRATION = 206,
  MSP_RESET_CONF = 208,
  MSP_SELECT_SETTING = 210,
  MSP_SET_HEAD = 211,
  MSP_BIND = 240,
  MSP_EEPROM_WRITE = 250
};

struct PID
{
  const static uint8_t type = 112;
  struct PIDitem
  {
    uint8_t P;
    uint8_t I;
    uint8_t D;
  };
  PIDitem PIDs[10];
} __attribute__((packed));

struct SetPID
{
  const static uint8_t type = 202;
  struct PIDitem
  {
    uint8_t P;
    uint8_t I;
    uint8_t D;
  };
  PIDitem PIDs[10];
} __attribute__((packed));

struct Ident
{
  const static uint8_t type = 100;
  uint8_t version;
  uint8_t multi_type;
  uint8_t msp_version;
  uint32_t capability;
} __attribute__((packed));

struct Status
{
  const static uint8_t type = 101;
  uint16_t cycle_time;
  uint16_t i2c_errors_count;
  uint16_t sensor;
  uint32_t flag;
  uint8_t current_set;
} __attribute__((packed));

struct RawIMU
{
  const static uint8_t type = 102;
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
  const static uint8_t type = 103;
  uint16_t servo[8];
} __attribute__((packed));

struct Motor
{
  const static uint8_t type = 104;
  uint16_t motor[8];
} __attribute__((packed));

struct RC
{
  const static uint8_t type = 105;
  uint16_t rcData[8];
} __attribute__((packed));

struct SetRawRC
{
  const static uint8_t type = 200;
  uint16_t rcData[8];
} __attribute__((packed));

struct Attitude
{
  const static uint8_t type = 108;
  int16_t angx;
  int16_t angy;
  int16_t heading;
} __attribute__((packed));

struct Altitude
{
  const static uint8_t type = 109;
  int32_t estAlt;
  int16_t vario;
} __attribute__((packed));

struct AccCalibration
{
  const static uint8_t type = 205;
} __attribute__((packed));
