#pragma once

#include <frc/WPILib.h>
using namespace frc;

#define LINE_SENSOR_COUNT 9


class LineSensor
{
public:

    LineSensor();
    ~LineSensor();
    
    void init();
    bool getLineSensor(int lineSensorNumber);
private:
    DigitalInput *digitalInputArray[LINE_SENSOR_COUNT];
};

