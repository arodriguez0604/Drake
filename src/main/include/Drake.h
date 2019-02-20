/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include "DalekDrive.h"
#include "MicroLidar.h"
#include "DalekShuffleboard.h"
#include "LineSensor.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
private:
  DalekDrive *m_drive;
  frc::Joystick *m_leftStick;
  frc::Joystick *m_rightStick;
  MicroLidar *microLidar;
  LineSensor *lineSensor;
  DalekShuffleboard *dalekShuffleboard;
};
