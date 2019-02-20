#include "DalekShuffleboard.h"

DalekShuffleboard::DalekShuffleboard(MicroLidar *microLidar, LineSensor *lineSensor) {
    this->microLidar = microLidar;
    this->lineSensor = lineSensor;
    init();
}
DalekShuffleboard::~DalekShuffleboard() {

}



void DalekShuffleboard::init() {
    

    for (int i = 0; i < LIDAR_COUNT; i++) {
        std::string temp = "Lidar " + std::to_string(i) + " Reset";
        frc::SmartDashboard::PutBoolean(temp, resetLidar[i]);
    }
    
    frc::SmartDashboard::PutBoolean("Allow Lidar Reset", calibrateLidar);
}

void DalekShuffleboard::continious() {
    calibrateLidar = frc::SmartDashboard::GetBoolean("Allow Lidar Reset", false);
    if (calibrateLidar) {
        for (int i = 0; i < LIDAR_COUNT; i++) {
            std::string temp = "Lidar " + std::to_string(i) + " Reset";
            resetLidar[i] = frc::SmartDashboard::GetBoolean(temp, false);
            if (resetLidar[i]) {
                //Callibrate Lidar Sensors
                resetLidar[i] = false;
                frc::SmartDashboard::PutBoolean(temp, resetLidar[i]);
            }
        }
    }

    for (int i = 0; i < LIDAR_COUNT; i++) {
        std::string temp = "Lidar Sensor: " + std::to_string(i);
        frc::SmartDashboard::PutNumber(temp, microLidar->GetMeasurement(i));
    }
    
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        std::string temp = "Line Sensor: "+ std::to_string(i);
        frc::SmartDashboard::PutBoolean(temp, lineSensor->getLineSensor(i));
    }
}