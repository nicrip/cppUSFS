#include <cstdint>
#include <vector>
#include "i2c.h"

#ifndef USFS_H
#define USFS_H

// Sensor-specific definitions
// LSM6DSM
#define LSM6DSM_GYRO_LPF_167        0x00
#define LSM6DSM_GYRO_LPF_314        0x02
#define LSM6DSM_GYRO_LPF_223        0x01
#define LSM6DSM_GYRO_LPF_655        0x03
#define LSM6DSM_GYRO_DLPF_CFG       LSM6DSM_GYRO_LPF_167
#define LSM6DSM_ACC_LPF_ODR_DIV2    0x00
#define LSM6DSM_ACC_LPF_ODR_DIV4    0x01
#define LSM6DSM_ACC_LPF_ODR_DIV9    0x02
#define LSM6DSM_ACC_LPF_ODR_DIV50   0x03
#define LSM6DSM_ACC_LPF_ODR_DIV100  0x04
#define LSM6DSM_ACC_LPF_ODR_DIV400  0x05
#define LSM6DSM_ACC_DLPF_CFG        LSM6DSM_ACC_LPF_ODR_DIV400
// LIS2MDL
#define LIS2MDL_MAG_LPF_ODR_DIV2    0x00
#define LIS2MDL_MAG_LPF_ODR_DIV4    0x01
#define LIS2MDL_MAG_DLPF_ODR_CFG    LIS2MDL_MAG_LPF_ODR_DIV4
// LPS22HB
#define LPS22HB_BARO_LPF_ODR_DIV2   0x00
#define LPS22HB_BARO_LPF_ODR_DIV9   0x01
#define LPS22HB_BARO_LPF_ODR_DIV20  0x02
#define LPS22HB_BARO_DLPF_ODR_CFG   LPS22HB_BARO_LPF_ODR_DIV2
// GENERIC
#define ACC_ODR_1660HZ              0xA6
#define ACC_ODR_834HZ               0x53
#define ACC_ODR_416HZ               0x29
#define ACC_ODR_208HZ               0x14
#define ACC_ODR_104HZ               0x0A
#define ACC_ODR_52HZ                0x05
#define ACC_ODR_26HZ                0x02
#define ACC_ODR_12HZ                0x01
#define ACC_ODR                     ACC_ODR_834HZ
#define GYRO_ODR_1660HZ             0xA6
#define GYRO_ODR_834HZ              0x53
#define GYRO_ODR_416HZ              0x29
#define GYRO_ODR_208HZ              0x14
#define GYRO_ODR_104HZ              0x0A
#define GYRO_ODR_52HZ               0x05
#define GYRO_ODR_26HZ               0x02
#define GYRO_ODR_12HZ               0x01
#define GYRO_ODR                    GYRO_ODR_834HZ
#define MAG_ODR_100HZ               0x64
#define MAG_ODR_50HZ                0x32
#define MAG_ODR_20HZ                0x14
#define MAG_ODR_10HZ                0x0A
#define MAG_ODR                     MAG_ODR_100HZ
#define BARO_ODR_75HZ               0x48
#define BARO_ODR_50HZ               0x32
#define BARO_ODR_25HZ               0x19
#define BARO_ODR_10HZ               0x0A
#define BARO_ODR_1HZ                0x01
#define BARO_ODR                    BARO_ODR_25HZ
#define QUAT_DIV_16HZ               0x0F
#define QUAT_DIV_10HZ               0x09
#define QUAT_DIV_8HZ                0x07
#define QUAT_DIV_5HZ                0x04
#define QUAT_DIV_4HZ                0x03
#define QUAT_DIV_2HZ                0x01
#define QUAT_DIV_1HZ                0x00
#define QUAT_DIV                    QUAT_DIV_8HZ
#define ACC_SCALE_2G                0x02
#define ACC_SCALE_4G                0x04
#define ACC_SCALE_8G                0x08
#define ACC_SCALE_16G               0x10
#define ACC_SCALE                   ACC_SCALE_8G
#define GYRO_SCALE_125DPS           0x7D
#define GYRO_SCALE_250DPS           0xFA
#define GYRO_SCALE_500DPS           0x1F4
#define GYRO_SCALE_1000DPS          0x3E8
#define GYRO_SCALE_2000DPS          0x7D0
#define GYRO_SCALE                  GYRO_SCALE_2000DPS
#define MAG_SCALE_4915UT            0x133
#define MAG_SCALE                   MAG_SCALE_4915UT
#define DPS_PER_COUNT               0.1525878906
#define SENTRAL_UT_PER_COUNT        0.0305176
#define G_PER_COUNT                 0.0004882813

// EEPROM definitions
#define EEPROM_DATA_ADDRESS         0x50

// USFS register definitions
#define USFS_ADDRESS                0x28
#define USFS_QX                     0x00
#define USFS_QY                     0x04
#define USFS_QZ                     0x08
#define USFS_QW                     0x0C
#define USFS_QTIME                  0x10
#define USFS_MX                     0x12
#define USFS_MY                     0x14
#define USFS_MZ                     0x16
#define USFS_MTIME                  0x18
#define USFS_AX                     0x1A
#define USFS_AY                     0x1C
#define USFS_AZ                     0x1E
#define USFS_ATIME                  0x20
#define USFS_GX                     0x22
#define USFS_GY                     0x24
#define USFS_GZ                     0x26
#define USFS_GTIME                  0x28
#define USFS_Baro                   0x2A
#define USFS_BaroTIME               0x2C
#define USFS_Temp                   0x2E
#define USFS_TempTIME               0x30
#define USFS_QRateDivisor           0x32
#define USFS_EnableEvents           0x33
#define USFS_HostControl            0x34
#define USFS_EventStatus            0x35
#define USFS_SensorStatus           0x36
#define USFS_SentralStatus          0x37
#define USFS_AlgorithmStatus        0x38
#define USFS_FeatureFlags           0x39
#define USFS_ParamAcknowledge       0x3A
#define USFS_SavedParamByte0        0x3B
#define USFS_SavedParamByte1        0x3C
#define USFS_SavedParamByte2        0x3D
#define USFS_SavedParamByte3        0x3E
#define USFS_ActualMagRate          0x45
#define USFS_ActualAccelRate        0x46
#define USFS_ActualGyroRate         0x47
#define USFS_ErrorRegister          0x50
#define USFS_AlgorithmControl       0x54
#define USFS_MagRate                0x55
#define USFS_AccelRate              0x56
#define USFS_GyroRate               0x57
#define USFS_BaroRate               0x58
#define USFS_TempRate               0x59
#define USFS_LoadParamByte0         0x60
#define USFS_LoadParamByte1         0x61
#define USFS_LoadParamByte2         0x62
#define USFS_LoadParamByte3         0x63
#define USFS_ParamRequest           0x64
#define USFS_ROMVersion1            0x70
#define USFS_ROMVersion2            0x71
#define USFS_RAMVersion1            0x72
#define USFS_RAMVersion2            0x73
#define USFS_ProductID              0x90
#define USFS_RevisionID             0x91
#define USFS_RunStatus              0x92
#define USFS_UploadAddress          0x94
#define USFS_UploadData             0x96
#define USFS_CRCHost                0x97
#define USFS_ResetRequest           0x9B
#define USFS_PassThruStatus         0x9E
#define USFS_PassThruControl        0xA0
#define USFS_ACC_LPF_BW             0x5B
#define USFS_GYRO_LPF_BW            0x5C
#define USFS_MAG_LPF_BW             0x5D
#define USFS_BARO_LPF_BW            0x5E
#define USFS_GP8                    0x3F
#define USFS_GP9                    0x40
#define USFS_GP10                   0x41
#define USFS_GP11                   0x42
#define USFS_GP12                   0x43
#define USFS_GP13                   0x44
#define USFS_GP20                   0x4B
#define USFS_GP21                   0x4C
#define USFS_GP22                   0x4D
#define USFS_GP23                   0x4E
#define USFS_GP24                   0x4F
#define USFS_GP36                   0x5B
#define USFS_GP37                   0x5C
#define USFS_GP38                   0x5D
#define USFS_GP39                   0x5E
#define USFS_GP40                   0x5F
#define USFS_GP50                   0x69
#define USFS_GP51                   0x6A
#define USFS_GP52                   0x6B
#define USFS_GP53                   0x6C
#define USFS_GP54                   0x6D
#define USFS_GP55                   0x6E
#define USFS_GP56                   0x6F

#define ACCEL_CAL                   1
#define WARM_START                  1

#define ACC_NORTH 0
#define ACC_EAST  1
#define ACC_DOWN  2
#define MAG_NORTH 0
#define MAG_EAST  1
#define MAG_DOWN  2

class USFS {
    protected:
        // i2c bus variables
        int bus;
        I2CDevice device;
        I2CDevice EEPROM_device;
        unsigned char buf[16];
        unsigned char rbuf[16];
        // i2c bus functions
        uint8_t readRegister(uint8_t subAddress);
        void readRegisters(uint8_t subAddress, uint8_t count);
        void writeRegister(uint8_t subAddress, uint8_t data);
        void EEPROMReadRegisters(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t* data, uint8_t count);
        void EEPROMWriteRegister(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t data);
        void EEPROMWriteRegisters(uint8_t subAddress_1, uint8_t subAddress_2, uint8_t* data, uint8_t count);

        void i2c_getSixRawADC(uint8_t add, uint8_t reg);
        float uint32_reg_to_float (uint8_t *buf);
        void float_to_bytes (float param_val, uint8_t *buf);
        void USFS_set_gyro_FS (uint16_t gyro_fs);
        void USFS_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs);
        void USFS_set_integer_param (uint8_t param, uint32_t param_val);
        void USFS_set_float_param (uint8_t param, float param_val);
        void USFS_acc_cal_upload();
        void USFS_set_WS_params();
        void USFS_get_WS_params();
        void WS_PassThroughMode();
        void WS_Resume();
        void readSenParams();
        void writeSenParams();
        void readAccelCal();
        void writeAccCal();

        int16_t accZero_max[3];
        int16_t accZero_min[3];
        uint8_t Sen_param[35][4];
        uint8_t Accel_Cal_valid = 0;
        uint8_t Sentral_WS_valid = 0;
        uint8_t eventStatus = 0;
    public:
        USFS();
        void ACC_getADC();
        void LIN_ACC_getADC();
        void ACC_Common();
        void Mag_getADC();
        void Save_Sentral_WS_params();
        void Gyro_getADC();
        void getQUAT();
        int16_t Baro_getPress();
        int16_t Baro_getTemp();
        void initSensors();
};

#endif
