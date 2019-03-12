#include "DalekShuffleboard.h"

DalekShuffleboard::DalekShuffleboard(MicroLidar *microLidar, LineSensor *lineSensor) 
{
#ifdef USE_LIDAR
    this->microLidar = microLidar;
#endif
    this->lineSensor = lineSensor;
    init();
}

DalekShuffleboard::~DalekShuffleboard()
{
}

void
DalekShuffleboard::init()
{
#ifdef USE_LIDAR
    #ifdef USE_CALLIBRATE
        for (int i = 0; i < LIDAR_COUNT; i++) {
            std::string temp = "Lidar " + std::to_string(i) + " Reset";
            frc::SmartDashboard::PutBoolean(temp, resetLidar[i]);
        }
        frc::SmartDashboard::PutBoolean("Allow Lidar Reset", calibrateLidar);
    #endif
#endif
}

void DalekShuffleboard::continious()
{
#ifdef USE_LIDAR
   #ifdef USE_CALLIBRATE
   calibrateLidar = frc::SmartDashboard::GetBoolean("Allow Lidar Reset", false);
    if (calibrateLidar) {
        // for (int i = 0; i < LIDAR_COUNT; i++) {
            std::string temp = "Lidar " + std::to_string(i) + " Reset";
            resetLidar[i] = frc::SmartDashboard::GetBoolean(temp, false);
            if (resetLidar[i]) {
                //Callibrate Lidar Sensors
                resetLidar[i] = false;
                frc::SmartDashboard::PutBoolean(temp, resetLidar[i]);
            }
        }
    }
    #endif
    for (int i = 0; i < LIDAR_COUNT; i++) {
        std::string temp = "Lidar Sensor: " + std::to_string(i);
        frc::SmartDashboard::PutNumber(temp, microLidar->GetMeasurement(i));
   }
#endif
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        std::string temp = "Line Sensor: "+ std::to_string(i);
        frc::SmartDashboard::PutBoolean(temp, lineSensor->getLineSensor(i));
    }
}