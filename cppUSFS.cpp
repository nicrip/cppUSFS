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
  device.page_bytes = 32;
  device.iaddr_bytes = 1;

  EEPROM_device.bus = bus;
  EEPROM_device.addr = EEPROM_DATA_ADDRESS;
  EEPROM_device.tenbit = 0;
  EEPROM_device.delay = 1;
  EEPROM_device.flags = 0;
  EEPROM_device.page_bytes = 32;
  EEPROM_device.iaddr_bytes = 2;

  bool upload = false;
  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  for (int attempts=0; attempts<100; ++attempts) {
      if (readRegister(USFS_SentralStatus) & 0x01) {
          upload = true;
          break;
      } else {
        writeRegister(USFS_ResetRequest, 0x01);
        std::this_thread::sleep_for(milliseconds(500));
      }
  }
  if (readRegister(USFS_SentralStatus) & 0x04) {
    upload = false;
    std::cout << "could not initialize the Sentral.. Exiting." << std::endl;
  }
  if (!upload) {
    std::cout << "Failed to upload firmware to device!" << std::endl;
  } else {
    std::cout << "Firmware upload successful." << std::endl;
  }
  std::cout << "Sentral status:" << +readRegister(USFS_SentralStatus) << " (should be 3)." << std::endl;

  #ifdef DEBUG
    std::cout << "Fetching EEPROM acceleration and warm-start calibration parameters..." << std::endl;
  #endif

  // Place SENtral in pass-through mode
  writeRegister(USFS_PassThruControl, 0x01);
  std::this_thread::sleep_for(milliseconds(100));
  STAT = readRegister(USFS_PassThruStatus);
  while(!(STAT & 0x01))
  {
    STAT = readRegister(USFS_PassThruStatus);
    std::this_thread::sleep_for(milliseconds(100));
  }

  // Fetch Accel Caldata from I2C EEPROM
  USFS::readAccelCal();

  // Fetch Warm Start data from I2C EEPROM
  USFS::readSenParams();

  // Cancel pass-through mode
  writeRegister(USFS_PassThruControl, 0x00);
  std::this_thread::sleep_for(milliseconds(100));
  STAT = readRegister(USFS_PassThruStatus);
  while((STAT & 0x01))
  {
    STAT = readRegister(USFS_PassThruStatus);
    std::this_thread::sleep_for(milliseconds(100));
  }
  #ifdef DEBUG
    std::cout << "Done." << std::endl;
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
    std::cout << "Checking and loading accelerometer calibration data if good...";
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
    std::cout << "Accelerometer calibration data is valid..." << std::endl;                                                                                          // Blink if accel cal'd true
  } else
  {
    std::cout << "Accelerometer calibration data is NOT valid. Defaults loaded..." << std::endl;
  }

  // Force initialize; reads Accel Cal data into static variable
  writeRegister(USFS_HostControl, 0x01);
  #ifdef DEBUG
    std::cout << "Done. Loading warm start parameters, modifying sensor ranges and data rates..." << std::endl;
  #endif
  std::this_thread::sleep_for(milliseconds(100));

  // Apply Warm Start Parameters
  if(WARM_START && Sentral_WS_valid == 1)
  {
    USFS::USFS_set_WS_params();
    std::cout << "Warm Start data loaded..." << std::endl;
  } else
  {
    std::cout << "Warm Start data NOT loaded." << std::endl;
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
  startSentral();

  // Enable interrupt to host upon certain events
  // Choose interrupts when: gyros updated (0x20), Sentral error (0x02) or Sentral reset (0x01)
  writeRegister(USFS_EnableEvents, 0x23);

  #ifdef DEBUG
    std::cout << "Done. Starting the Sentral..." << std::endl;
  #endif

  // Start the Sentral
  startSentral();
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

  Start_time = std::chrono::high_resolution_clock::now();
}

void USFS::startSentral()
{
  writeRegister(USFS_AlgorithmControl, 0x20);
}

void USFS::i2c_getSixRawADC(uint8_t reg)
{
  readRegisters(reg, &rawADC[0], 6);
}

void USFS::getQUAT()
{
  uint8_t rawData[18];

  readRegisters(USFS_QX, &rawData[0], 18);
  qt[1] = uint32_reg_to_float (&rawData[0]);
  qt[2] = uint32_reg_to_float (&rawData[4]);
  qt[3] = uint32_reg_to_float (&rawData[8]);
  qt[0] = uint32_reg_to_float (&rawData[12]);
  QT_Timestamp = ((int16_t)(rawData[17]<<8) | rawData[16]);
}

int16_t USFS::Baro_getPress()
{
  uint8_t rawData[2];

  readRegisters(USFS_Baro, &rawData[0], 2);
  return (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
}

int16_t USFS::Baro_getTemp()
{
  uint8_t rawData[2];

  readRegisters(USFS_Temp, &rawData[0], 2);
  return (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);
}

void USFS::Gyro_getADC()
{
  USFS::i2c_getSixRawADC(USFS_GX);
  GYRO_ORIENTATION( ((int16_t)(rawADC[1]<<8) | rawADC[0]),                                                                      // Range: +/- 32768; +/- 2000 deg/sec
                    ((int16_t)(rawADC[3]<<8) | rawADC[2]),
                    ((int16_t)(rawADC[5]<<8) | rawADC[4]));
}

void USFS::ACC_getADC()
{
  USFS::i2c_getSixRawADC(USFS_AX);
  acc[0] = ((int16_t)(rawADC[1]<<8) | rawADC[0]);                                                                    // Scale: 2048cts = 1g
  acc[1] = ((int16_t)(rawADC[3]<<8) | rawADC[2]);
  acc[2] = ((int16_t)(rawADC[5]<<8) | rawADC[4]);
  ACC_ORIENTATION(acc[0],
                  acc[1],
                  acc[2]);

  // If Accel Cal active, assign ACC_CAL_ORIENTATION definition to generate calibration values
  if(calibratingA > 0)
  {
    ACC_CAL_ORIENTATION(acc[0],
                        acc[1],
                        acc[2]);
  }
}

void USFS::LIN_ACC_getADC()
{
  USFS::i2c_getSixRawADC(USFS_GP8);
  accLIN[0] = ((int16_t)(rawADC[1]<<8) | rawADC[0]);                                                                 // Scale: 2048cts = 1g
  accLIN[1] = ((int16_t)(rawADC[3]<<8) | rawADC[2]);
  accLIN[2] = ((int16_t)(rawADC[5]<<8) | rawADC[4]);
}

void USFS::Mag_getADC()
{
  USFS::i2c_getSixRawADC(USFS_MX);
  MAG_ORIENTATION( (int16_t)((rawADC[1]<<8) | rawADC[0]),
                   (int16_t)((rawADC[3]<<8) | rawADC[2]),
                   (int16_t)((rawADC[5]<<8) | rawADC[4]));
}

void USFS::ACC_Common()
{
  if (calibratingA == 512)
  {
    // Tell the Sentral to send unscaled sensor data
    writeRegister(USFS_AlgorithmControl, 0x02);
    std::this_thread::sleep_for(milliseconds(100));

    // Re-read Acc data in raw data mode
    USFS::ACC_getADC();

  }
  if (calibratingA > 0)
  {
    for (uint8_t axis = 0; axis < 3; axis++)
    {
      // Raw Acc data from Sentral is scaled according to sensor range selected; divide by appropriate factor of two to get 1g=2048cts
      if ((acc_calADC[axis]/(0x10/ACC_SCALE) > 1024))
      {
        // Sum up 512 readings
        a_acc[axis] += acc_calADC[axis]/(0x10/ACC_SCALE);
      }
      if ((acc_calADC[axis]/(0x10/ACC_SCALE)) < -1024)
      {
        b_acc[axis] += acc_calADC[axis]/(0x10/ACC_SCALE);
      }
      // Clear global variables for next reading
      acc_calADC[axis] = 0;
    }

    // Calculate averages, and store values in EEPROM at end of calibration
    if (calibratingA == 1)
    {
      for (uint8_t axis = 0; axis < 3; axis++)
      {
        if (a_acc[axis]>>9 > 1024)
        {
          accZero_max[axis] = a_acc[axis]>>9;
        }
        if (b_acc[axis]>>9 < -1024)
        {
          accZero_min[axis] = b_acc[axis]>>9;
        }
        // Clear global variables for next time
        a_acc[axis] = 0;
        b_acc[axis] = 0;
      }

      // Save accZero to I2C EEPROM
      // Put the Sentral in pass-thru mode
      USFS::WS_PassThroughMode();

      // Store accelerometer calibration data to the M24512DFM I2C EEPROM
      USFS::writeAccCal();

      // Take Sentral out of pass-thru mode and re-start algorithm
      // Also resumes sending calibrated sensor data to the output registers
      USFS::WS_Resume();
    }
    calibratingA--;
  }
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
  startSentral();
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
  startSentral();
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
  startSentral();
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
  startSentral();
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
  startSentral();
}

void USFS::USFS_get_WS_params()
{
  uint8_t param = 1;
  uint8_t STAT;

  writeRegister(USFS_ParamRequest, param);
  std::this_thread::sleep_for(milliseconds(100));

  // Request parameter transfer procedure
  writeRegister(USFS_AlgorithmControl, 0x80);
  std::this_thread::sleep_for(milliseconds(100));

   // Check the parameter acknowledge register and loop until the result matches parameter request byte
  STAT = readRegister(USFS_ParamAcknowledge);
  while(!(STAT==param))
  {
    STAT = readRegister(USFS_ParamAcknowledge);
  }

  // Parameter is the decimal value with the MSB set low (default) to indicate a paramter read processs
  Sen_param[0][0] = readRegister(USFS_SavedParamByte0);
  Sen_param[0][1] = readRegister(USFS_SavedParamByte1);
  Sen_param[0][2] = readRegister(USFS_SavedParamByte2);
  Sen_param[0][3] = readRegister(USFS_SavedParamByte3);

  for(uint8_t i=1; i<35; i++)
  {
    param = (i+1);
    writeRegister(USFS_ParamRequest, param);
    std::this_thread::sleep_for(milliseconds(100));

    // Check the parameter acknowledge register and loop until the result matches parameter request byte
    STAT = readRegister(USFS_ParamAcknowledge);
    while(!(STAT==param))
    {
      STAT = readRegister(USFS_ParamAcknowledge);
    }
    Sen_param[i][0] = readRegister(USFS_SavedParamByte0);
    Sen_param[i][1] = readRegister(USFS_SavedParamByte1);
    Sen_param[i][2] = readRegister(USFS_SavedParamByte2);
    Sen_param[i][3] = readRegister(USFS_SavedParamByte3);
  }
  // Parameter request = 0 to end parameter transfer process
  writeRegister(USFS_ParamRequest, 0x00);
  std::this_thread::sleep_for(milliseconds(100));

  // Re-start algorithm
  startSentral();;
  std::this_thread::sleep_for(milliseconds(100));
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

void USFS::WS_PassThroughMode()
{
  uint8_t stat = 0;

  // Put SENtral in standby mode
  writeRegister(USFS_AlgorithmControl, 0x01);
  std::this_thread::sleep_for(milliseconds(100));

  // Place SENtral in pass-through mode
  writeRegister(USFS_PassThruControl, 0x01);
  std::this_thread::sleep_for(milliseconds(100));
  stat = readRegister(USFS_PassThruStatus);
  std::this_thread::sleep_for(milliseconds(100));
  while(!(stat & 0x01))
  {
    stat = readRegister(USFS_PassThruStatus);
    std::this_thread::sleep_for(milliseconds(5));
  }
}

void USFS::WS_Resume()
{
  uint8_t stat = 0;

  // Cancel pass-through mode
  writeRegister(USFS_PassThruControl, 0x00);
  std::this_thread::sleep_for(milliseconds(100));
  stat = readRegister(USFS_PassThruStatus);
  while((stat & 0x01))
  {
    stat = readRegister(USFS_PassThruStatus);
    std::this_thread::sleep_for(milliseconds(5));
  }

  // Re-start algorithm
  startSentral();
  std::this_thread::sleep_for(milliseconds(100));
  stat = readRegister(USFS_AlgorithmStatus);
  while((stat & 0x01))
  {
    stat = readRegister(USFS_AlgorithmStatus);
    std::this_thread::sleep_for(milliseconds(5));
  }

  // Read event status register to clear interrupt
  eventStatus = readRegister(USFS_EventStatus);
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
    accZero_max[axis] = ((int16_t)(data[(2*axis + 1)]<<8) | data[2*axis]);
    accZero_min[axis] = ((int16_t)(data[(2*axis + 7)]<<8) | data[(2*axis + 6)]);
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
    readRegisters(subAddress, rbuf, 1);
    return rbuf[0];
}

void USFS::writeRegister(uint8_t subAddress, uint8_t data)
{
    buf[0] = data;
    if (i2c_write(&device, subAddress, buf, 1) != 1) {
      std::cout << "WRITE ERROR" << std::endl;
    }
}

void USFS::readRegisters(uint8_t subAddress, uint8_t* data, uint8_t count)
{
    if (i2c_read(&device, subAddress, data, count) != count) {
      std::cout << "READ ERROR" << std::endl;
    }
}

void USFS::EEPROMWriteRegister(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t data)
{
    uint16_t subAddress = ((uint16_t)subAddress_1 << 8) | subAddress_2;
    buf[0] = data;
    if (i2c_ioctl_write(&EEPROM_device, subAddress, buf, 1) != 1) {
      std::cout << "WRITE ERROR" << std::endl;
    }
}

void USFS::EEPROMReadRegisters(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t* data, uint8_t count)
{
    uint16_t subAddress = ((uint16_t)subAddress_1 << 8) | subAddress_2;
    if (i2c_ioctl_read(&EEPROM_device, subAddress, data, count) != count) {
      std::cout << "READ ERROR" << std::endl;
    }
}

void USFS::EEPROMWriteRegisters(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t* data, uint8_t count)
{
    uint16_t subAddress = ((uint16_t)subAddress_1 << 8) | subAddress_2;
    for (uint8_t i = 0; i < count; i++) {
      buf[i] = data[i];
    }
    if (i2c_ioctl_write(&EEPROM_device, subAddress, buf, count) != count) {
      std::cout << "WRITE ERROR" << std::endl;
    }
}

void USFS::computeIMU()
{
  float a11, a21, a31, a32, a33;
  float yaw;
  static float buff_roll = 0.0f, buff_pitch = 0.0f, buff_heading = 0.0f;

  // Pass-thru for future filter experimentation
  accSmooth[0] = accADC[0];
  accSmooth[1] = accADC[1];
  accSmooth[2] = accADC[2];

  USFS::getQUAT();

  // Only five elements of the rotation matrix are necessary to calculate the three Euler angles
  a11 = qt[0]*qt[0] + qt[1]*qt[1] - qt[2]*qt[2] - qt[3]*qt[3];
  a21 = 2.0f*(qt[0]*qt[3] + qt[1]*qt[2]);
  a31 = 2.0f*(qt[1]*qt[3] - qt[0]*qt[2]);
  a32 = 2.0f*(qt[0]*qt[1] + qt[2]*qt[3]);
  a33 = qt[0]*qt[0] - qt[1]*qt[1] - qt[2]*qt[2] + qt[3]*qt[3];

  // Pass-thru for future filter experimentation
  buff_roll    = (atan2(a32, a33))*(57.2957795f);                                                          // Roll Right +ve
  buff_pitch   = -(asin(a31))*(57.2957795f);                                                               // Pitch Up +ve
  buff_heading = (atan2(a21, a11))*(57.2957795f);                                                          // Yaw CW +ve

  angle[0] = buff_roll;
  angle[1] = buff_pitch;
  yaw      = buff_heading;
  heading  = yaw + mag_declination;
  if(heading < 0.0f) heading += 360.0f;                                                                    // Convert heading to 0 - 360deg range
  Curr_time = std::chrono::high_resolution_clock::now();
  Diff_time = duration_cast<microseconds>(Curr_time - Start_time);
  TimeStamp = Diff_time.count()/1000000.0f;
}

void USFS::Save_Sentral_WS_params()
{
      USFS::USFS_get_WS_params();

      // Put the Sentral in pass-thru mode
      USFS::WS_PassThroughMode();

      // Store WarmStart data to the M24512DFM I2C EEPROM
      USFS::writeSenParams();

      // Take Sentral out of pass-thru mode and re-start algorithm
      USFS::WS_Resume();
}

void USFS::FetchEventStatus()
{
    eventStatus = (int)readRegister(USFS_EventStatus);

    if(eventStatus & 0x04) Quat_flag = 1;
    if(eventStatus & 0x20) Gyro_flag = 1;
    if(eventStatus & 0x10) Acc_flag  = 1;
    if(eventStatus & 0x08) Mag_flag  = 1;
    if(eventStatus & 0x40) Baro_flag = 1;
    algoStatus = (int)readRegister(USFS_AlgorithmStatus);
}

void USFS::FetchSentralData()
{
  if(Gyro_flag) {
    Gyro_getADC();
    for(uint8_t i=0; i<3; i++) {
      gyroData[i] = (float)gyroADC[i]*DPS_PER_COUNT;
    }
    Gyro_flag = 0;
  }
  
  if(Quat_flag) {
    computeIMU();
    Quat_flag = 0;
  }
  
  if(Acc_flag) {
    ACC_getADC();
    ACC_Common();
    for(uint8_t i=0; i<3; i++) {
      accData[i] = (float)accADC[i]*G_PER_COUNT;
      LINaccData[i] = (float)accLIN[i]*G_PER_COUNT;
    }
    Acc_flag = 0;
  }

  if(Mag_flag) {
    Mag_getADC();
    for(uint8_t i=0; i<3; i++) {
      magData[i] = (float)magADC[i]*SENTRAL_UT_PER_COUNT;
    }
    Mag_flag = 0;
  }

  if(Baro_flag) {
    rawPressure    = Baro_getPress();
    pressure       = (float)rawPressure*0.01f +1013.25f;                                       // Pressure in mBar
    rawTemperature = Baro_getTemp();
    temperature    = (float)rawTemperature*0.01;                                               // Temperature in degrees C
    Baro_flag      = 0;
  }
}