
#include "VL53L0X.h"
#include "vl53l0x_api_calibration.h"

VL53L0X::VL53L0X(int _DigGpioIdx, VL53L0X_DeviceModes _DeviceMode)
{
    dprint("Initializing VL53L0X for GPIO: " << _DigGpioIdx << " Mode: " << (uint32_t)_DeviceMode);
    // Assigne the I2C address by basing off the default address + 1 and adding GPIO index
    DigGpioIdx = _DigGpioIdx;
    I2CAddr = VL53L0X_DEFAULT_I2C_ADDR + 1 + DigGpioIdx;

    switch (_DeviceMode) {
        case VL53L0X_DEVICEMODE_SINGLE_RANGING:
            dprint("DeviceMode set to single ranging for GPIO:" << _DigGpioIdx);
            DeviceMode = _DeviceMode;
        case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
            dprint("DeviceMode set to continuous ranging for GPIO:" << _DigGpioIdx);
            DeviceMode = _DeviceMode;
            break;
        default:
            dprint("!!!DeviceMode slected is unsupported!!! Setting to Single ranging for GPIO:" << _DigGpioIdx);
            DeviceMode = VL53L0X_DEVICEMODE_SINGLE_RANGING;
            break;
    }
}

VL53L0X::VL53L0X(int _DigGpioIdx)
{
    DigGpioIdx = _DigGpioIdx;
    I2CAddr = VL53L0X_DEFAULT_I2C_ADDR + 1 + DigGpioIdx;
    DeviceMode = VL53L0X_DEVICEMODE_SINGLE_RANGING;
    dprint("short Initializing VL53L0X for GPIO: " << _DigGpioIdx << " Mode: " << (uint32_t)DeviceMode);
}

bool VL53L0X::Init()
{
    Init(I2CAddr);
}

bool VL53L0X::Init(int _I2CAddr, int _DigGpioIdx)
{
    DigGpioIdx = _DigGpioIdx;
    I2CAddr = _I2CAddr;
    Init(I2CAddr);
}

bool VL53L0X::Init(int I2CAddr)
{
    bool rval;
    rval = _Init(I2CAddr);
    Reset();
    SetDeviceMode();

    return rval;    
}

void VL53L0X::SetDeviceMode()
{
    dprint("Setting device mode= " << (uint32_t)DeviceMode);
    Status = VL53L0X_SetDeviceMode(&MyDevice, DeviceMode);
    print_pal_error(Status);
}

bool VL53L0X::_Init(int I2cAddr)
{
    // Initialize Comms
    BlockingTimer Timer;
    VL53L0X_Version_t Version;
    VL53L0X_DeviceInfo_t DeviceInfo;

    DataSelect = 0;

    // We need to use a temp i2c device to get comms started using the default addess
    VL53L0X_Dev_t TempDevice;

    // Set the i2c address of our default object to the device default
    TempDevice.I2cDevAddr = VL53L0X_DEFAULT_I2C_ADDR;

    // Set the desired i2c address for the device we are initializing
    MyDevice.I2cDevAddr = I2cAddr;

    // Setup comms to the temporary device
    if(_i2c_init(&TempDevice, (char *)"/dev/i2c-2", TempDevice.I2cDevAddr) < 0) 
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }

    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);

    // Using the temp device, send the command to change its adress
    //   NOTE: The address used by the ST library here is an 8-bit address, not 7. Hence the *2;
    if((Status = VL53L0X_SetDeviceAddress(&TempDevice, (uint32_t)MyDevice.I2cDevAddr * 2)) != VL53L0X_ERROR_NONE)
    {   
        dprint("VL53L0X_SetDeviceAddress failed" );
        print_pal_error(Status);
        return false;
    } 

    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);

    // Now we need to initialize comms to the i2c device we'll use
    if(_i2c_init(&MyDevice, (char *)"/dev/i2c-2", (uint32_t)MyDevice.I2cDevAddr) < 0) 
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        return false;
    }

    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);

    if((Status = VL53L0X_DataInit(&MyDevice)) != VL53L0X_ERROR_NONE)
    {
        dprint("VL53L0X_DataInit FAILED\n");
        // Data initialization
        print_pal_error(Status);
        return false;
    }

    if((Status = VL53L0X_GetVersion(&Version)) != VL53L0X_ERROR_NONE)
    {
        dprint("VL53L0X_GetVersion failed" );
        return false;
    }

    /*
    *  Verify the version of the VL53L0X API running in the firmware
    */

    if(Status == VL53L0X_ERROR_NONE)
    {
        if( Version.major != VERSION_REQUIRED_MAJOR ||
            Version.minor != VERSION_REQUIRED_MINOR ||
            Version.build != VERSION_REQUIRED_BUILD )
        {
            //dprint("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
            //    Version.major, Version.minor, Version.build, Version.revision,
            //    VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
            return false;
        }
    }
    else 
    {
        dprint("Bad VL53L0X_GetVersion ");
        return false;
    }

    if((Status = VL53L0X_get_device_info(&MyDevice, &DeviceInfo)) == VL53L0X_ERROR_NONE)
    {
        dprint("VL53L0X_GetDeviceInfo:");
        dprint("Device Name : " << DeviceInfo.Name);
        dprint("Device Type : " << DeviceInfo.Type);
        dprint("Device ID : " << DeviceInfo.ProductId);
        dprint("ProductRevisionMajor : " << (uint32_t)DeviceInfo.ProductRevisionMajor);
        dprint("ProductRevisionMinor : " << (uint32_t)DeviceInfo.ProductRevisionMinor);
        
        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) 
        {
            dprint("Error expected cut 1.1 but found cut " << DeviceInfo.ProductRevisionMajor << "." << DeviceInfo.ProductRevisionMinor);
            Status = VL53L0X_ERROR_NOT_SUPPORTED;
            print_pal_error(Status);
            return false;
        }
        dprint("VL53L0X Micro Lidar Init successful on i2c Addr= " << (uint32_t)MyDevice.I2cDevAddr);
    }
    else
    {
        print_pal_error(Status);
        return false;
    }

    return true;
}

void VL53L0X::Reset()
{ 
    int i;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    //void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData);
    //char buf[VL53L0X_MAX_STRING_LENGTH];
    //uint8_t RangeStatus;

    /*
    * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
    */

    //RangeStatus = pRangingMeasurementData->RangeStatus;

    //VL53L0X_GetRangeStatusString(RangeStatus, buf);
    //dprintf("Range Status: %i : %s\n", RangeStatus, buf);
    
    if(Status == VL53L0X_ERROR_NONE)
    {
    dprint("Call of VL53L0X_StaticInit " << MyDevice.fd);
    Status = VL53L0X_StaticInit(&MyDevice); // Device Initialization
    print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
    dprint("Call of VL53L0X_PerformRefCalibration");
    Status = VL53L0X_PerformRefCalibration(&MyDevice,&VhvSettings, &PhaseCal); // Device Initialization
    print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
    dprint("Call of VL53L0X_PerformRefSpadManagement");
    Status = VL53L0X_PerformRefSpadManagement(&MyDevice,&refSpadCount, &isApertureSpads); // Device Initialization
    dprint("refSpadCount = " << refSpadCount);
    dprint("isApertureSpads = " << isApertureSpads);
    print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
    // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
    dprint("Call of VL53L0X_SetDeviceMode");
    Status = VL53L0X_SetDeviceMode(&MyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
    print_pal_error(Status);
    }

    // Enable/Disable Sigma and Signal check

/*   if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetSequenceStepEnable(MyDevice,VL53L0X_SEQUENCESTEP_DSS, 1);
    }*/	

    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckEnable(&MyDevice,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                (FixPoint1616_t)(0.1*65536));
    }			
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetLimitCheckValue(&MyDevice,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                (FixPoint1616_t)(60*65536));			
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&MyDevice,33000);
    } 

    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice,VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
    if (Status == VL53L0X_ERROR_NONE) 
    {
        Status = VL53L0X_SetVcselPulsePeriod(&MyDevice,VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    }
}

void VL53L0X::StartMeasurement()
{
    SetDeviceMode();
    dprint("StartMeasurement GPIO=" << DigGpioIdx);
    Status = VL53L0X_StartMeasurement(&MyDevice);
    print_pal_error(Status);
}

void VL53L0X::StopMeasurement()
{
    dprint("StopMeasurement GPIO=" << DigGpioIdx);
    Status = VL53L0X_StopMeasurement(&MyDevice);
    print_pal_error(Status);
}

int VL53L0X::GetMeasurement()
{
    switch (DeviceMode) {
        case VL53L0X_DEVICEMODE_SINGLE_RANGING:
            return MeasureOnce_MM();
        case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
            return ValidatedMeasurement_get();
    }
}

int VL53L0X::ValidatedMeasurement_get()
{
    int Data = Measurement;
    //int Data = RangingMeasurementData[DataSelect_get()].RangeMilliMeter;

    if (Status == VL53L0X_ERROR_NONE)
    {      
        if(Data < 1000)
            return Data;
        else
            return 9999;  
    } 
    else  
        return 9001;
}

int VL53L0X::MeasureOnce_MM()
{
    if(Status == VL53L0X_ERROR_NONE || BYPASS_PRE_MEASUREMENT_STATUS_CHECK)
    {
        Status = VL53L0X_PerformSingleRangingMeasurement(&MyDevice,&RangingMeasurementData[DataSelect_get()]);

        dprint("measurement status= " << (uint32_t)Status << "data: " << (uint32_t)RangingMeasurementData[DataSelect_get()].RangeMilliMeter);
        //print_pal_error(Status);

        return ValidatedMeasurement_get();
    }   
    return 9002; 
}

int VL53L0X::PollMeasurementData()
{
    uint32_t NewDataReady=0;
    VL53L0X_RangingMeasurementData_t    Data;
    int count=0;

    //Status = VL53L0X_GetMeasurementDataReady(&MyDevice, &NewDataReady);
    Status = VL53L0X_GetInterruptMaskStatus(&MyDevice, &NewDataReady);
    if(NewDataReady == VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY && 
        Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetRangingMeasurementData(&MyDevice, &Data);
//        dprint("data= " << Data.RangeMilliMeter << " Status: " << (int)Status << " Select: " << (int) DataSelect << ":" << (int) peek_DataSelect());
        if(Status == VL53L0X_ERROR_NONE) {
            Measurement = Data.RangeMilliMeter;
        }
        else    
        {
            StopMeasurement();
            VL53L0X_ClearInterruptMask(&MyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            StartMeasurement();
            dprint("Error during ContinuousMeasurement: " << (int)Status);
            return 0;
        }

        do {
            Status = VL53L0X_ClearInterruptMask(&MyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        } while(count++ < 3 && Status != VL53L0X_ERROR_NONE);
        return 1;
    } else if(NewDataReady != 0x01)
    {
        return 0;
    } else
    {
        // Error
        StopMeasurement();
        do {
            Status = VL53L0X_ClearInterruptMask(&MyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        } while(count++ < 3 && Status != VL53L0X_ERROR_NONE);
        StartMeasurement();
        return 0;
    }
    
}

void VL53L0X::Calibrate(int CalibrationDistance)
{
    int Offset_micrometer;

    Status = VL53L0X_perform_offset_calibration(&MyDevice, (unsigned)CalibrationDistance, &Offset_micrometer);
}