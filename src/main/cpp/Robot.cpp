/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


// check comments in arm cpp lines 1&2
// check comments in dalekdrive cpp line 280

#include <iostream>
#include "Drake.h"

void
Robot::RobotInit() 
{
    m_drive      = new DalekDrive(1, 2, 3, 4, DalekDrive::driveType::kMecanum);
    m_leftStick  = new frc::Joystick(1);
    //m_rightStick = new frc::Joystick(2);
    m_xbox       = new frc::XboxController(3);
    m_dPad[R]    = new frc::POVButton(*m_xbox, 0);
    m_dPad[T]    = new frc::POVButton(*m_xbox, 90);
    m_dPad[L]    = new frc::POVButton(*m_xbox, 180);
    m_dPad[B]    = new frc::POVButton(*m_xbox, 270);


    microLidar = new MicroLidar("/dev/i2c-2", MicroLidar::CONTINUOUS_MEASURE_MODE);
    for (int i = 0; i < LIDAR_COUNT; i++) {
        microLidar->Add(i);
    }
    microLidar->InitSensors();
    microLidar->StartMeasurements();
    lineSensor = new LineSensor();
    dalekShuffleboard = new DalekShuffleboard(microLidar, lineSensor);
}

void
Robot::RobotPeriodic() 
{
   microLidar->PollDevices();
   dalekShuffleboard->continious();
}

void
Robot::AutonomousInit() 
{
}

void
Robot::AutonomousPeriodic()
{
}

void
Robot::TeleopInit() 
{
}

void
Robot::TeleopPeriodic()
{
    m_drive->Cartesian(*m_leftStick, 0.0);
    m_claw->Tick(m_xbox);
    m_arm->Tick(m_xbox, m_dPad);

    //Motor Voltage values
    m_arm->printVoltage(m_leftStick);
    m_claw->printVoltage();
}

void
Robot::TestPeriodic()
{

}

#ifndef RUNNING_FRC_TESTS
int
main() 
{ 
  return frc::StartRobot<Robot>();
}
#endif
