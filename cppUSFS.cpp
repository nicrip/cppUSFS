#include <iostream>
#include <sstream>
#include <math.h>
#include <chrono>
#include <thread>
#include <ctime>
#include "cppUSFS.h"

using namespace std::chrono;

#define DEBUG 1;

USFS::USFS() {
}

void USFS::initSensors()
{
  uint8_t STAT;
  int count = 0;

  if ((bus = i2c_open("/dev/i2c-2")) == -1) {
    std::cout << "i2c bus could not be opened... Exiting." << std::endl;
    exit(0);
  }
  device.bus = bus;
  device.addr = USFS_ADDRESS;
  device.tenbit = 0;
  device.delay = 1;
  device.flags = 0;
  device.page_bytes = 16;
  device.iaddr_bytes = 1;

  EEPROM_device.bus = bus;
  EEPROM_device.addr = EEPROM_DATA_ADDRESS;
  EEPROM_device.tenbit = 0;
  EEPROM_device.delay = 1;
  EEPROM_device.flags = 0;
  EEPROM_device.page_bytes = 16;
  EEPROM_device.iaddr_bytes = 2;

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  for (int attempts=0; attempts<10; ++attempts) {
      if (readRegister(USFS_SentralStatus) & 0x01) {
          if(readRegister(USFS_SentralStatus) & 0x01) { }
          if(readRegister(USFS_SentralStatus) & 0x02) { }
          if(readRegister(USFS_SentralStatus) & 0x04) {
              std::cout << "could not initialize SENtral.. Exiting." << std::endl;
          }
          if(readRegister(USFS_SentralStatus) & 0x08) { }
          if(readRegister(USFS_SentralStatus) & 0x10) {
              std::cout << "could not initialize SENtral.. Exiting." << std::endl;
          }
          break;
      }
      writeRegister(USFS_ResetRequest, 0x01);
      std::this_thread::sleep_for(milliseconds(500));
  }
  if (readRegister(USFS_SentralStatus) & 0x04) {
      std::cout << "could not initialize SENtral.. Exiting." << std::endl;
  }

  #ifdef DEBUG
    std::cout << "Sentral firmware loaded. Fetching Accel Cal and WS parameters..." << std::endl;
  #endif

  // Place SENtral in pass-through mode
  writeRegister(USFS_PassThruControl, 0x01);
  std::this_thread::sleep_for(milliseconds(5));
  STAT = readRegister(USFS_PassThruStatus);
  std::this_thread::sleep_for(milliseconds(5));
  while(!(STAT & 0x01))
  {
    STAT = readRegister(USFS_PassThruStatus);
    std::this_thread::sleep_for(milliseconds(5));
  }

  // Fetch Accel Caldata from I2C EEPROM
  USFS::readAccelCal();

  // Fetch Warm Start data from I2C EEPROM
  USFS::readSenParams();

  // Cancel pass-through mode
  writeRegister(USFS_PassThruControl, 0x00);
  std::this_thread::sleep_for(milliseconds(5));
  STAT = readRegister(USFS_PassThruStatus);
  while((STAT & 0x01))
  {
    STAT = readRegister(USFS_PassThruStatus);
    std::this_thread::sleep_for(milliseconds(5));
  }
  #ifdef DEBUG
    std::cout << "Done..." << std::endl;
  #endif

  // Print Accel Cal raw data for inspection
  #ifdef DEBUG
    std::cout << "Acceleromater Calibration Data:" << std::endl;
    std::cout << "X-acc max: ";
    std::cout << accZero_max[0] << std::endl;
    std::cout << "Y-acc max: ";
    std::cout << accZero_max[1] << std::endl;
    std::cout << "Z-acc max: ";
    std::cout << accZero_max[2] << std::endl;
    std::cout << "X-acc min: ";
    std::cout << accZero_min[0] << std::endl;
    std::cout << "Y-acc min: ";
    std::cout << accZero_min[1] << std::endl;
    std::cout << "Z-acc min: ";
    std::cout << accZero_min[2] << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Checking/loading Acc Cal data...";
    std::cout << "" << std::endl;
  #endif

  // Be sure Sentral is in "Idle" state
  writeRegister(USFS_HostControl, 0x00);

  // Load Accel Cal
  // Check that the Acc Cal values are valid and will not crash the Sentral
  Accel_Cal_valid = 1;
  for (uint8_t i = 0; i < 3; i++)
  {
    if ((accZero_min[i] < -2240) || (accZero_min[i] > -1800)) Accel_Cal_valid = 0;
    if ((accZero_max[i] < 1800) || (accZero_max[i] > 2240)) Accel_Cal_valid = 0;
  }

  // If Accel Cal data bogus, null data loaded instead
  USFS::USFS_acc_cal_upload();
  if(ACCEL_CAL && Accel_Cal_valid)
  {
    std::cout << "Acc Cal data valid..." << std::endl;                                                                                          // Blink if accel cal'd true
  } else
  {
    std::cout << "Acc Cal data invalid! Defaults loaded..." << std::endl;
  }

  // Force initialize; reads Accel Cal data into static variable
  writeRegister(USFS_HostControl, 0x01);
  #ifdef DEBUG
    std::cout << "Done. Loading Warm Start Parameters, modifying sensor ranges and data rates..." << std::endl;
  #endif
  std::this_thread::sleep_for(milliseconds(20));

  // Apply Warm Start Parameters
  if(WARM_START && Sentral_WS_valid == 1)
  {
    USFS::USFS_set_WS_params();
    std::cout << "Warm Start data loaded..." << std::endl;
  } else
  {
    std::cout << "Warm Start data NOT loaded!" << std::endl;
  }

  // Set Sensor LPF bandwidth. MUST BE DONE BEFORE SETTING ODR's
  writeRegister(USFS_ACC_LPF_BW, LSM6DSM_ACC_DLPF_CFG);       // Accelerometer
  writeRegister(USFS_GYRO_LPF_BW, LSM6DSM_GYRO_DLPF_CFG);     // Gyroscope
  writeRegister(USFS_MAG_LPF_BW, LIS2MDL_MAG_DLPF_ODR_CFG);   // Magnetometer
  writeRegister(USFS_BARO_LPF_BW, LPS22HB_BARO_DLPF_ODR_CFG); // Baro

  // Set accel/gyro/mage desired ODR rates
  writeRegister(USFS_AccelRate, ACC_ODR);
  writeRegister(USFS_GyroRate, GYRO_ODR);
  writeRegister(USFS_MagRate, MAG_ODR);
  writeRegister(USFS_QRateDivisor, QUAT_DIV);

  // ODR + 10000000b to activate the eventStatus bit for the barometer...
  writeRegister(USFS_BaroRate, (0x80 + BARO_ODR));

  // Configure operating mode
  // Output scaled sensor data (Quaternion convention NED)
  writeRegister(USFS_AlgorithmControl, 0x00);

  // Enable interrupt to host upon certain events
  // Choose interrupts when: gyros updated (0x20), Sentral error (0x02) or Sentral reset (0x01)
  writeRegister(USFS_EnableEvents, 0x23);

  #ifdef DEBUG
    std::cout << "Done. Starting the Sentral..." << std::endl;
  #endif

  // Start the Sentral
  writeRegister(USFS_AlgorithmControl, 0x00);
  #ifdef DEBUG
    std::cout << "Done. Loading algorithm tuning parameters..." << std::endl;
  #endif

  // Perform final Sentral alogorithm parameter modifications
  USFS::USFS_set_integer_param(0x49, 0x00);                                                                                // Disable "Stillness" mode
  USFS::USFS_set_integer_param(0x48, 0x01);                                                                                // Set Gbias_mode to 1
  USFS::USFS_set_mag_acc_FS(MAG_SCALE, ACC_SCALE);                                                                         // Set magnetometer/accelerometer full-scale ranges
  USFS::USFS_set_gyro_FS(GYRO_SCALE);                                                                                      // Set gyroscope full-scale range
  USFS::USFS_set_float_param(0x3B, 0.0f);                                                                                  // Param 59 Mag Transient Protect off (0.0)

  // Choose interrupt events: Gyros updated (0x20), Sentral error (0x02) or Sentral reset (0x01)
  writeRegister(USFS_EnableEvents, 0x23);

  // Read event status register
  eventStatus = readRegister(USFS_EventStatus);

  #ifdef DEBUG
    std::cout << "Sentral initialization complete!" << std::endl;
    std::cout << "" << std::endl;
  #endif
}

void USFS::USFS_set_gyro_FS (uint16_t gyro_fs)
{
  uint8_t bytes[4], STAT;

  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  writeRegister(USFS_LoadParamByte0, bytes[0]);
  writeRegister(USFS_LoadParamByte1, bytes[1]);
  writeRegister(USFS_LoadParamByte2, bytes[2]);
  writeRegister(USFS_LoadParamByte3, bytes[3]);

  // Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  writeRegister(USFS_ParamRequest, 0xCB);

  // Request parameter transfer procedure
  writeRegister(USFS_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readRegister(USFS_ParamAcknowledge);
  while(!(STAT==0xCB))
  {
    STAT = readRegister(USFS_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeRegister(USFS_ParamRequest, 0x00);

  // Re-start algorithm
  writeRegister(USFS_AlgorithmControl, 0x00);
}

void USFS::USFS_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs)
{
  uint8_t bytes[4], STAT;

  bytes[0] = mag_fs & (0xFF);
  bytes[1] = (mag_fs >> 8) & (0xFF);
  bytes[2] = acc_fs & (0xFF);
  bytes[3] = (acc_fs >> 8) & (0xFF);
  writeRegister(USFS_LoadParamByte0, bytes[0]);
  writeRegister(USFS_LoadParamByte1, bytes[1]);
  writeRegister(USFS_LoadParamByte2, bytes[2]);
  writeRegister(USFS_LoadParamByte3, bytes[3]);

  //Parameter 74; 0xCA is 74 decimal with the MSB set high to indicate a paramter write processs
  writeRegister(USFS_ParamRequest, 0xCA);

  // Request parameter transfer procedure
  writeRegister(USFS_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readRegister(USFS_ParamAcknowledge);
  while(!(STAT==0xCA))
  {
    STAT = readRegister(USFS_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process
  writeRegister(USFS_ParamRequest, 0x00);

  // Re-start algorithm
  writeRegister(USFS_AlgorithmControl, 0x00);
}

void USFS::USFS_set_integer_param (uint8_t param, uint32_t param_val)
{
  uint8_t bytes[4], STAT;

  bytes[0] = param_val & (0xFF);
  bytes[1] = (param_val >> 8) & (0xFF);
  bytes[2] = (param_val >> 16) & (0xFF);
  bytes[3] = (param_val >> 24) & (0xFF);

  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeRegister(USFS_LoadParamByte0, bytes[0]);
  writeRegister(USFS_LoadParamByte1, bytes[1]);
  writeRegister(USFS_LoadParamByte2, bytes[2]);
  writeRegister(USFS_LoadParamByte3, bytes[3]);
  writeRegister(USFS_ParamRequest, param);

  // Request parameter transfer procedure
  writeRegister(USFS_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readRegister(USFS_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readRegister(USFS_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process=
  writeRegister(USFS_ParamRequest, 0x00);

  // Re-start algorithm
  writeRegister(USFS_AlgorithmControl, 0x00);
}

void USFS::USFS_set_float_param (uint8_t param, float param_val)
{
  uint8_t bytes[4], STAT;

  USFS::float_to_bytes (param_val, &bytes[0]);

  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeRegister(USFS_LoadParamByte0, bytes[0]);
  writeRegister(USFS_LoadParamByte1, bytes[1]);
  writeRegister(USFS_LoadParamByte2, bytes[2]);
  writeRegister(USFS_LoadParamByte3, bytes[3]);
  writeRegister(USFS_ParamRequest, param);

  // Request parameter transfer procedure
  writeRegister(USFS_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readRegister(USFS_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readRegister(USFS_ParamAcknowledge);
  }
  // Parameter request = 0 to end parameter transfer process=
  writeRegister(USFS_ParamRequest, 0x00);

  // Re-start algorithm
  writeRegister(USFS_AlgorithmControl, 0x00);
}

void USFS::USFS_set_WS_params()
{
  uint8_t param = 1;
  uint8_t STAT;

  // Parameter is the decimal value with the MSB set high to indicate a paramter write processs
  param = param | 0x80;
  writeRegister(USFS_LoadParamByte0, Sen_param[0][0]);
  writeRegister(USFS_LoadParamByte1, Sen_param[0][1]);
  writeRegister(USFS_LoadParamByte2, Sen_param[0][2]);
  writeRegister(USFS_LoadParamByte3, Sen_param[0][3]);
  writeRegister(USFS_ParamRequest, param);

  // Request parameter transfer procedure
  writeRegister(USFS_AlgorithmControl, 0x80);

  // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readRegister(USFS_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readRegister(USFS_ParamAcknowledge);
  }
  for(uint8_t i=1; i<35; i++)
  {
    param = (i+1) | 0x80;
    writeRegister(USFS_LoadParamByte0, Sen_param[i][0]);
    writeRegister(USFS_LoadParamByte1, Sen_param[i][1]);
    writeRegister(USFS_LoadParamByte2, Sen_param[i][2]);
    writeRegister(USFS_LoadParamByte3, Sen_param[i][3]);
    writeRegister(USFS_ParamRequest, param);

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readRegister(USFS_ParamAcknowledge);
    while(!(STAT==param))
    {
      STAT = readRegister(USFS_ParamAcknowledge);
    }
  }
  // Parameter request = 0 to end parameter transfer process=
  writeRegister(USFS_ParamRequest, 0x00);

  // Re-start algorithm=
  writeRegister(USFS_AlgorithmControl, param);
}


void USFS::USFS_acc_cal_upload()
{
  int64_t big_cal_num;
  union
  {
    int16_t cal_num;
    unsigned char cal_num_byte[2];
  };

  if(!ACCEL_CAL || !Accel_Cal_valid)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    //NORTH SCALE
    big_cal_num = (4096000000/(accZero_max[ACC_NORTH] - accZero_min[ACC_NORTH])) - 1000000;
    cal_num = (int16_t)big_cal_num;
  }
  writeRegister(USFS_GP36, cal_num_byte[0]);
  writeRegister(USFS_GP37, cal_num_byte[1]);

  if(!ACCEL_CAL || !Accel_Cal_valid)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
   // EAST SCALE
   big_cal_num = (4096000000/(accZero_max[ACC_EAST] - accZero_min[ACC_EAST])) - 1000000;
   cal_num = (int16_t)big_cal_num;
  }
  writeRegister(USFS_GP38, cal_num_byte[0]);
  writeRegister(USFS_GP39, cal_num_byte[1]);

  if(!ACCEL_CAL || !Accel_Cal_valid)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
   // DOWN SCALE
   big_cal_num = (4096000000/(accZero_max[ACC_DOWN] - accZero_min[ACC_DOWN])) - 1000000;
    cal_num = (int16_t)big_cal_num;
  }
  writeRegister(USFS_GP40, cal_num_byte[0]);
  writeRegister(USFS_GP50, cal_num_byte[1]);

  if(!ACCEL_CAL || !Accel_Cal_valid)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    // NORTH OFFSET
    big_cal_num = (((accZero_max[ACC_NORTH] - 2048) + (accZero_min[ACC_NORTH] + 2048))*100000)/4096;
    cal_num = (int16_t)big_cal_num;
  }
  writeRegister(USFS_GP51, cal_num_byte[0]);
  writeRegister(USFS_GP52, cal_num_byte[1]);

  if(!ACCEL_CAL || !Accel_Cal_valid)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    // EAST OFFSET
    big_cal_num = (((accZero_max[ACC_EAST] - 2048) + (accZero_min[ACC_EAST] + 2048))*100000)/4096;
    cal_num = (int16_t)big_cal_num;
  }
  writeRegister(USFS_GP53, cal_num_byte[0]);
  writeRegister(USFS_GP54, cal_num_byte[1]);

  if(!ACCEL_CAL || !Accel_Cal_valid)
  {
    cal_num_byte[0] = 0;
    cal_num_byte[1] = 0;
  } else
  {
    // DOWN OFFSET
    big_cal_num = (((accZero_max[ACC_DOWN] - 2048) + (accZero_min[ACC_DOWN] + 2048))*100000)/4096;
    cal_num = (int16_t)big_cal_num;
  }
  writeRegister(USFS_GP55, cal_num_byte[0]);
  writeRegister(USFS_GP56, cal_num_byte[1]);
}

void USFS::readSenParams()
{
  uint8_t data[140];
  uint8_t paramnum;

  // Write in 16 byte chunks because the ESP8266 Wire buffer is only 32 bytes
  // 140 bytes total; first 128:
  for(uint8_t i = 0; i < 8; i++)
  {
    EEPROMReadRegisters(0x80, (0x00 + 16*i), &data[(16*i)], 16);
  }

  // Last 12 bytes
  EEPROMReadRegisters(0x80, 0x80, &data[128], 12);
  for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
  {
    for (uint8_t i= 0; i < 4; i++)
    {
      Sen_param[paramnum][i] = data[(paramnum*4 + i)];
    }
  }
  EEPROMReadRegisters(0x80, 0x98, &Sentral_WS_valid, 1);

  // If byte 0x98 is 10101010b (0xaa) that means valid WS data is in the EEPROM
  if(Sentral_WS_valid == 0xaa)
  {
    Sentral_WS_valid = 1;
  }else
  {
    Sentral_WS_valid = 0;
  }
}

void USFS::writeSenParams()
{
  uint8_t data[140];
  uint8_t paramnum;

  for (paramnum = 0; paramnum < 35; paramnum++) // 35 parameters
  {
    for (uint8_t i= 0; i < 4; i++)
    {
      data[(paramnum*4 + i)] = Sen_param[paramnum][i];
    }
  }

  // Write in 16 byte chunks because the ESP8266 Wire buffer is only 32 bytes
  // 140 bytes total; first 128:
  for(uint8_t i = 0; i < 8; i++)
  {
    EEPROMWriteRegisters(0x80, (0x00 + 16*i), &data[(16*i)], 16);
  }

  // Last 12 bytes
  EEPROMWriteRegisters(0x80, 0x80, &data[128], 12);

  // Valid Warm Start byte; if save of WS parameters is successful write 10101010b (0xaa) to byte 0x98 (Free from 0x99 to 0xFF)
  Sentral_WS_valid = 0xaa;
  EEPROMWriteRegister(0x80, 0x98, Sentral_WS_valid);
  Sentral_WS_valid = 0x00;
}

void USFS::readAccelCal()
{
  uint8_t data[12];
  uint8_t axis;

  EEPROMReadRegisters(0x80, 0x8c, &data[0], 12);
  for (axis = 0; axis < 3; axis++)
  {
    accZero_max[axis] = ((int16_t)(rbuf[(2*axis + 1)]<<8) | rbuf[2*axis]);
    accZero_min[axis] = ((int16_t)(rbuf[(2*axis + 7)]<<8) | rbuf[(2*axis + 6)]);
  }
}

void USFS::writeAccCal()
{
  uint8_t data[12];
  uint8_t axis;
  for (axis = 0; axis < 3; axis++)
  {
    data[2*axis] = (accZero_max[axis] & 0xff);
    data[(2*axis + 1)] = (accZero_max[axis] >> 8);
    data[(2*axis + 6)] = (accZero_min[axis] & 0xff);
    data[(2*axis + 7)] = (accZero_min[axis] >> 8);
  }
  EEPROMWriteRegisters(0x80, 0x8c, data, 12);
}

void USFS::float_to_bytes (float param_val, uint8_t *buf)
{
  union
  {
    float f;
    uint8_t comp[sizeof(float)];
  } u;

  u.f = param_val;
  for (uint8_t i=0; i < sizeof(float); i++)
  {
    buf[i] = u.comp[i];
  }

  // Convert to LITTLE ENDIAN
  for (uint8_t i=0; i < sizeof(float); i++)
  {
    buf[i] = buf[(sizeof(float)-1) - i];
  }
}

float USFS::uint32_reg_to_float (uint8_t *buf)
{
    union {
        uint32_t ui32;
        float f;
    } u;

    u.ui32 = (((uint32_t)buf[0]) +
            (((uint32_t)buf[1]) <<  8) +
            (((uint32_t)buf[2]) << 16) +
            (((uint32_t)buf[3]) << 24));
    return u.f;
}

uint8_t USFS::readRegister(uint8_t subAddress)
{
    readRegisters(subAddress, 1);
    return rbuf[0];
}

void USFS::writeRegister(uint8_t subAddress, uint8_t data)
{
    buf[0] = data;
    if (i2c_write(&device, subAddress, buf, 1) != 1) {
      std::cout << "WRITE ERROR" << std::endl;
    }
}

void USFS::readRegisters(uint8_t subAddress, uint8_t count)
{
    if (i2c_read(&device, subAddress, rbuf, count) != count) {
      std::cout << "READ ERROR" << std::endl;
    }
}

void USFS::EEPROMWriteRegister(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t data)
{
    uint16_t subAddress = ((uint16_t)subAddress_2 << 8) | subAddress_1;
    buf[0] = data;
    if (i2c_ioctl_write(&EEPROM_device, subAddress, buf, 1) != 1) {
      std::cout << "WRITE ERROR" << std::endl;
    }
}

void USFS::EEPROMReadRegisters(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t* data, uint8_t count)
{
    uint16_t subAddress = ((uint16_t)subAddress_2 << 8) | subAddress_1;
    if (i2c_ioctl_read(&EEPROM_device, subAddress, data, count) != count) {
      std::cout << "READ ERROR" << std::endl;
    }
}

void USFS::EEPROMWriteRegisters(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t* data, uint8_t count)
{
    uint16_t subAddress = ((uint16_t)subAddress_2 << 8) | subAddress_1;
    for (uint8_t i = 0; i < count; i++) {
      buf[i] = data[i];
    }
    if (i2c_ioctl_write(&EEPROM_device, subAddress, buf, count) != count) {
      std::cout << "WRITE ERROR" << std::endl;
    }
}

int main(int argc, char *argv[])
{

}
