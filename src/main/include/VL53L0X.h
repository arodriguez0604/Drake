/*
 *
 */

#pragma once

#include <iostream>

#include "Timer.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#define VL53L0X_SETTLE_TIMEDELAY 0.01 // seconds
#define VL53L0X_DEFAULT_I2C_ADDR 0x29

#define VERSION_REQUIRED_MAJOR 1
#define VERSION_REQUIRED_MINOR 0
#define VERSION_REQUIRED_BUILD 1

// Uncomment to enable logging
//#define DEBUG_MICRO_LIDAR 
#ifdef dprint 
  #undef dprint
#endif
#ifdef DEBUG_MICRO_LIDAR
 #define dprint(strm) {std::cout << strm << std::endl;}
#else
 #define dprint(strm) {}
#endif

// Set this to 1 to avoid a device status check prior to trying a measurement
#define BYPASS_PRE_MEASUREMENT_STATUS_CHECK 1

class VL53L0X
{
public:
    VL53L0X(int _DigGpioIdx, VL53L0X_DeviceModes _DeviceMode);
    VL53L0X(int _DigGpioIdx);

    bool Init();
    bool Init(int I2cAddr, int _DigGpioIdx);
    bool Init(int I2cAddr);
    int GetDigGpioIdx() { return DigGpioIdx; }
    void SetDeviceMode();
    void StartMeasurement();
    void StopMeasurement();
    int GetMeasurement();
    int MeasureOnce_MM();
    // Intended to be called from an asynchronous polling mechanism
    // This is required for continuous measurement mode
    int Status_get() { return Status; }
    int PollMeasurementData();
    void Calibrate(int CalibrationDistance);

private:
    bool _Init(int I2cAddr);
    void Reset();

    void print_pal_error(VL53L0X_Error Status)
    {
    #ifdef DEBUG_MICRO_LIDAR
        char buf[VL53L0X_MAX_STRING_LENGTH];
        VL53L0X_GetPalErrorString(Status, buf);
        dprint("API Status: " << Status << " : " << buf );
    #endif
    }
    
    int _i2c_init(VL53L0X_Dev_t *DeviceP, char *i2c_dev, int i2c_addr)
    {
        DeviceP->fd = VL53L0X_i2c_init(i2c_dev, i2c_addr);  
        if(DeviceP->fd < 0)
        {
            dprint("_i2c_init failed on " << i2c_dev << " at addr=" << i2c_addr );
            return -1;
        }
        dprint("i2c_init successful on bus " << i2c_dev << " at fd=" << DeviceP->fd << " and addr=" << i2c_addr );
        return 1;
    }

    int DataSelect_get() { return DataSelect; }
    void next_DataSelect() { DataSelect = peek_DataSelect(); }
    int peek_DataSelect() { return DataSelect_get() ^ 1; }
    int ValidatedMeasurement_get();
    
private:
    int DigGpioIdx;
    int I2CAddr;
    VL53L0X_Error Status; // = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData[2];
    int Measurement;
    int DataSelect;
    VL53L0X_DeviceModes DeviceMode;
};