#include "LineSensor.h"

LineSensor::LineSensor()
{
    init();
}

LineSensor::~LineSensor()
{
}

void
LineSensor::init() 
{
    digitalInputArray[0] = new frc::DigitalInput(10);
    digitalInputArray[1] = new frc::DigitalInput(11);
    digitalInputArray[2] = new frc::DigitalInput(12);
    digitalInputArray[3] = new frc::DigitalInput(13);
    digitalInputArray[4] = new frc::DigitalInput(18);
    digitalInputArray[5] = new frc::DigitalInput(19);
    digitalInputArray[6] = new frc::DigitalInput(20);
    digitalInputArray[7] = new frc::DigitalInput(21);
    digitalInputArray[8] = new frc::DigitalInput(22);
}

bool
LineSensor::getLineSensor(int lineSensorNumber)
{
    return !digitalInputArray[lineSensorNumber]->Get();
}