/*

    Class implementation for USFS in master mode

   Copyright (C) 2018 Simon D. Levy

   Adapted from

     https://raw.githubusercontent.com/kriswiner/Teensy_Flight_Controller/master/MPU9250_BMP280

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <sstream>
#include <math.h>
#include <chrono>
#include <thread>
#include <ctime>
#include "USFS_Master.h"

using namespace std::chrono;

USFS_Master::USFS_Master(uint8_t magRate, uint16_t accelRate, uint16_t gyroRate, uint8_t baroRate, uint8_t qRateDivisor)
{
    _magRate = magRate;
    _accelRate = accelRate;
    _gyroRate = gyroRate;
    _baroRate = baroRate;
    _qRateDivisor = qRateDivisor;
}

const char * USFS_Master::getErrorString(void)
{
    return _usfs.getErrorString();
}

bool USFS_Master::begin()
{
    // Fail immediately if unable to upload EEPROM
    if (!_usfs.begin()) return false;

    std::cout << 1 << std::endl;

    // Enter USFS initialized state
    _usfs.setRunDisable();// set SENtral in initialized state to configure registers
    std::cout << 2 << std::endl;
    _usfs.setMasterMode();
    std::cout << 3 << std::endl;
    _usfs.setRunEnable();
    std::cout << 4 << std::endl;
    _usfs.setRunDisable();// set SENtral in initialized state to configure registers
    std::cout << 5 << std::endl;

    // Setup LPF bandwidth (BEFORE setting ODR's)
    _usfs.setAccelLpfBandwidth(0x03); // 41Hz
    std::cout << 6 << std::endl;
    _usfs.setGyroLpfBandwidth(0x03);  // 41Hz
    std::cout << 7 << std::endl;

    // Set accel/gyro/mage desired ODR rates
    _usfs.setQRateDivisor(_qRateDivisor-1);
    std::cout << 8 << std::endl;
    _usfs.setMagRate(_magRate);
    std::cout << 9 << std::endl;
    _usfs.setAccelRate(_accelRate/10);
    std::cout << 10 << std::endl;
    _usfs.setGyroRate(_gyroRate/10);
    std::cout << 11 << std::endl;
    _usfs.setBaroRate(0x80 | _baroRate); // 0x80 = enable bit
    std::cout << 12 << std::endl;

    // Configure operating modeA
    _usfs.algorithmControlReset();// read scale sensor data
    std::cout << 13 << std::endl;

    // Enable interrupt to host upon certain events:
    // quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    _usfs.enableEvents(0x07);
    std::cout << 14 << std::endl;

    // Enable USFS run mode
    _usfs.setRunEnable();// set SENtral in normal run mode
    std::cout << 15 << std::endl;
    std::this_thread::sleep_for(milliseconds(100));
    std::cout << 16 << std::endl;

    // Disable stillness mode
    _usfs.setIntegerParam (0x49, 0x00);
    std::cout << 17 << std::endl;

    // Success
    return _usfs.getSensorStatus() ? false : true;
}

void USFS_Master::checkEventStatus(void)
{
    // Check event status register, way to check data ready by checkEventStatusing rather than interrupt
    _eventStatus = _usfs.getEventStatus(); // reading clears the register

}

bool USFS_Master::gotError(void)
{
    if (_eventStatus & 0x02) {

        return true;
    }

    return false;
}

bool USFS_Master::gotQuaternion(void)
{
    return _eventStatus & 0x04;
}

bool USFS_Master::gotMagnetometer(void)
{
    return _eventStatus & 0x08;
}

bool USFS_Master::gotAccelerometer(void)
{
    return _eventStatus & 0x10;
}

bool USFS_Master::gotGyrometer(void)
{
    return _eventStatus & 0x20;
}

bool USFS_Master::gotBarometer(void)
{
    return _eventStatus & 0x40;
}

void USFS_Master::readQuaternion(float & qw, float & qx, float & qy, float &qz)
{
    _usfs.readQuaternion(qw, qx, qy, qz);
}

void USFS_Master::readThreeAxis(uint8_t regx, float & x, float & y, float & z, float scale)
{
    int16_t xx=0, yy=0, zz=0;

    _usfs.readThreeAxis(regx, xx, yy, zz);

    x = xx * scale;
    y = yy * scale;
    z = zz * scale;
}

void USFS_Master::readAccelerometer(float & ax, float & ay, float & az)
{
    readThreeAxis(USFS::AX, ax, ay, az, 0.000488);
}

void USFS_Master::readGyrometer(float & gx, float & gy, float & gz)
{
    readThreeAxis(USFS::GX, gx, gy, gz, 0.153);
}

void USFS_Master::readMagnetometer(float & mx, float & my, float & mz)
{
    readThreeAxis(USFS::MX, mx, my, mz, 0.305176);
}

void USFS_Master::readBarometer(float & pressure, float & temperature)
{
    _usfs.readBarometer(pressure, temperature);
}

int main(int argc, char *argv[])
{
  static const uint8_t  MAG_RATE       = 100;  // Hz
  static const uint16_t ACCEL_RATE     = 200;  // Hz
  static const uint16_t GYRO_RATE      = 200;  // Hz
  static const uint8_t  BARO_RATE      = 50;   // Hz
  static const uint8_t  Q_RATE_DIVISOR = 3;    // 1/3 gyro rate

  USFS_Master usfs = USFS_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);
  // if (!usfs.begin()) {
  //     while (true) {
  //         std::cout << usfs.getErrorString() << std::endl;
  //     }
  // }
  bool test;
  test = usfs.begin();
  std::cout << test << std::endl;

  std::cout << "!!!!!!!!!!!!!!!!!!" << std::endl;

  while(1) {
    usfs.checkEventStatus();

    if (usfs.gotError()) {
        // std::cout << "ERROR:" << std::endl;
        // std::cout << usfs.getErrorString() << std::endl;
        continue;
    }

    if (usfs.gotQuaternion()) {

        float qw, qx, qy, qz;

        usfs.readQuaternion(qw, qx, qy, qz);

        float roll  = atan2(2.0f * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
        float pitch = -asin(2.0f * (qx * qz - qw * qy));
        float yaw   = atan2(2.0f * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);

        pitch *= 180.0f / M_PI;
        yaw   *= 180.0f / M_PI;
        yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        if(yaw < 0) yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
        roll  *= 180.0f / M_PI;

        std::cout << "Quaternion Roll, Pitch, Yaw: ";
        std::cout << roll;
        std::cout << ", ";
        std::cout << pitch;
        std::cout << ", ";
        std::cout << yaw << std::endl;
    }

    if (usfs.gotAccelerometer()) {
        float ax, ay, az;
        usfs.readAccelerometer(ax, ay, az);

        std::cout << "Accel: ";
        std::cout << ax;
        std::cout << ", ";
        std::cout << ay;
        std::cout << ", ";
        std::cout << az << std::endl;
    }

    if (usfs.gotGyrometer()) {
        float gx, gy, gz;
        usfs.readGyrometer(gx, gy, gz);

        std::cout << gx;
        std::cout << "Gyro: ";
        std::cout << gx;
        std::cout << ", ";
        std::cout << gy;
        std::cout << ", ";
        std::cout << gz << std::endl;
    }

    if (usfs.gotMagnetometer()) {

        float mx, my, mz;
        usfs.readMagnetometer(mx, my, mz);

        std::cout << "Mag: ";
        std::cout << mx;
        std::cout << ", ";
        std::cout << my;
        std::cout << ", ";
        std::cout << mz << std::endl;
    }

     /*
       Or define output variable according to the Android system, where
       heading (0 to 360) is defined by the angle between the y-axis and True
       North, pitch is rotation about the x-axis (-180 to +180), and roll is
       rotation about the y-axis (-90 to +90) In this systen, the z-axis is
       pointing away from Earth, the +y-axis is at the "top" of the device
       (cellphone) and the +x-axis points toward the right of the device.
     */

    if (usfs.gotBarometer())
    {
        float temperature, pressure;

        usfs.readBarometer(pressure, temperature);

        std::cout << "Baro:" << std::endl;
        std::cout << "  Altimeter temperature = ";
        std::cout <<  temperature;
        std::cout << " C" << std::endl;
        std::cout << "  Altimeter pressure = ";
        std::cout << pressure;
        std::cout << " mbar" << std::endl;
        float altitude = (1.0f - powf(pressure / 1013.25f, 0.190295f)) * 44330.0f;
        std::cout << "  Altitude = ";
        std::cout << altitude;
        std::cout << " m\n" << std::endl;
    }

  }
}
