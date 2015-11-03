#include "serial/mspdata.h"
#include <ostream>

std::ostream& operator<<(std::ostream& s, const Ident& d)
{
    return s 
        << "Version: " << static_cast<int>(d.version) << '\n'
        << "Type   : " << static_cast<int>(d.multi_type) << '\n';
    // TODO: add protocol version and capabilities
}

std::ostream& operator<<(std::ostream& s, const Status& d)
{
    return s 
        << "Cycle time: " << d.cycle_time << '\n'
        << "I2C errors: " << d.i2c_errors_count << '\n';
    // TODO: add other fields
}

std::ostream& operator<<(std::ostream& s, const RawIMU& d)
{
    return s 
        << "Acc: " << d.accx << ", " << d.accy << ", " << d.accz << '\n'
        << "Gyr: " << d.gyrx << ", " << d.gyry << ", " << d.gyrz << '\n'
        << "Mag: " << d.magx << ", " << d.magy << ", " << d.magz << '\n';
}

std::ostream& operator<<(std::ostream& s, const Servo& d)
{
    for (unsigned int i = 0; i < 8; ++i)
        s << "Servo " << i << ":" << d.servo[i] << '\n';
    return s;
}

std::ostream& operator<<(std::ostream& s, const Motor& d)
{
    for (unsigned int i = 0; i < 8; ++i)
        s << "Motor " << i << ":" << d.motor[i] << '\n';
    return s;
}

std::ostream& operator<<(std::ostream& s, const RC& d)
{
    for (unsigned int i = 0; i < 8; ++i)
        s << "RC " << i << ":" << d.rcData[i] << '\n';
    return s;
}

std::ostream& operator<<(std::ostream& s, const SetRawRC& d)
{
    for (unsigned int i = 0; i < 8; ++i)
        s << "RC " << i << ":" << d.rcData[i] << '\n';
    return s;
}

std::ostream& operator<<(std::ostream& s, const Attitude& d)
{
    return s
        << "AngX: " << d.angx << '\n'
        << "AngY: " << d.angy << '\n'
        << "Head: " << d.heading << '\n';
}

std::ostream& operator<<(std::ostream& s, const Altitude& d)
{
    return s
        << "Alt: " << d.estAlt << '\n'
        << "Var: " << d.vario << '\n';
}

std::ostream& operator<<(std::ostream& s, const AccCalibration& d)
{
    return s << "Acc calibration done\n";
}
