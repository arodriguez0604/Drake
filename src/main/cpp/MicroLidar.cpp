#include "MicroLidar.h"

MicroLidar::MicroLidar(std::string _I2CBusName, MeasurementMode _MeasureMode)
{
    dprint("New MicroLidar on " << I2CBusName << "Mode: " << _MeasureMode);
    I2CBusName  = _I2CBusName;
    MeasureMode = _MeasureMode;
    DeviceCount = 0;
}

MicroLidar::~MicroLidar()
{
    for(int i=0; i < DeviceCount; i++)
    {
        delete Devices[DeviceList[i]];
    }
}

// Add a single device
void
MicroLidar::Add(int DigGpioIdx)
{
    VL53L0X_DeviceModes DeviceMode;

    switch (MeasureMode) {
        case SINGLE_MEASURE_MODE:
            DeviceMode = VL53L0X_DEVICEMODE_SINGLE_RANGING;
            break;
        case CONTINUOUS_MEASURE_MODE:
            DeviceMode = VL53L0X_DEVICEMODE_CONTINUOUS_RANGING;
            break;
    }
    // Add new device
    DeviceList[DeviceCount] = DigGpioIdx;
    // Increment our count
    DeviceCount++;
    // Create a new device
    Devices[DigGpioIdx] = new VL53L0X(DigGpioIdx, DeviceMode);
    DigGpio[DigGpioIdx] = new frc::DigitalOutput(DigGpioIdx);
}

// Init all sensors that were Add'ed
void 
MicroLidar::InitSensors()
{
    BlockingTimer Timer;

    // We need to reset all the sensors and then hold each in reset
    SetAllGpio(true);
    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);
    SetAllGpio(false);
    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);
    SetAllGpio(true);
    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);
    SetAllGpio(false);
    Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);

    // Now we are ready to start
    for(int i=0; i < DeviceCount; i++) {
        DigGpio[DeviceList[i]]->Set(true);
        Timer.wait_BLOCKING(VL53L0X_SETTLE_TIMEDELAY);
        Devices[DeviceList[i]]->Init();
    }

    StartMeasurements();
}

void 
MicroLidar::StartMeasurements()
{
    if(MeasureMode == SINGLE_MEASURE_MODE)
        return;
    //Start measurement
    for(int i=0; i < DeviceCount; i++) {
        Devices[DeviceList[i]]->StartMeasurement();
    }
}

void
MicroLidar::StopMeasurements()
{
    if(MeasureMode == SINGLE_MEASURE_MODE)
        return;

    for(int i=0; i < DeviceCount; i++) {
        Devices[DeviceList[i]]->StopMeasurement();
    }
}

// Get the measurement for a single device
int
MicroLidar::GetMeasurement(int DigGpioIdx)
{
    return Devices[DigGpioIdx]->GetMeasurement();
}

void 
MicroLidar::Calibrate(int DigGpioIdx)
{
    Devices[DigGpioIdx]->StopMeasurement();
    Devices[DigGpioIdx]->Calibrate(CALIBRATION_DISTANCE_MM);
    Devices[DigGpioIdx]->StartMeasurement();
}

void 
MicroLidar::SetAllGpio(bool State)
{
    for(int i=0; i < DeviceCount; i++) {
        SetOneGpio(i, State);
    }
}

void
MicroLidar::SetOneGpio(int DeviceIdx, bool State)
{
    DigGpio[DeviceList[DeviceIdx]]->Set(State);
}

void
MicroLidar::PollDevices()
{
    for(int i=0; i < DeviceCount; i++) {
        //dprint("Polling: " << i);
        Devices[DeviceList[i]]->PollMeasurementData();
    }
}
