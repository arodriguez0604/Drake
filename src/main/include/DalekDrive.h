/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
 * Utility class for handling Robot drive 
 */
#pragma once

#include <Drake.h>

#define CAN_TIMEOUT 100
#define STALL_LIMIT 60
#define FREE_LIMIT  2
#define RAMP_RATE   0.5
#define LEFT        0
#define RIGHT       1
#define FRONT       0
#define REAR        1
#define NUM_MOTORS_PER_SIDE 2

class DalekDrive {
  public:
	enum class driveType { kMecanum, kDifferential };
	DalekDrive(int leftMotorChannel, int leftSlaveMotorChannel,
            int rightMotorChannel, int rightSlaveMotorChannel, driveType t);
	DalekDrive(rev::CANSparkMax* leftMotor, rev::CANSparkMax* leftSlaveMotor,
            rev::CANSparkMax* rightMotor, rev::CANSparkMax* rightSlaveMotor, driveType t);
	DalekDrive(rev::CANSparkMax& leftMotor, rev::CANSparkMax& leftSlaveMotor,
            rev::CANSparkMax& rightMotor, rev::CANSparkMax& rightSlaveMotor, driveType t);

	~DalekDrive();

	void TankDrive(frc::GenericHID* leftStick, frc::GenericHID* rightStick,
            bool squaredInputs = true);
	void TankDrive(frc::GenericHID& leftStick, frc::GenericHID& rightStick,
            bool squaredInputs = true);
	void TankDrive(double leftValue, double rightValue,
            bool squaredInputs = true);

	void ArcadeDrive(frc::GenericHID* stick, bool squaredInputs = true);
	void ArcadeDrive(frc::GenericHID& stick, bool squaredInputs = true);
	void ArcadeDrive(double moveValue, double rotateValue,
            bool squaredInputs = true);

	void Polar(frc::GenericHID* leftStick, frc::GenericHID* rightStick);
	void Polar(frc::GenericHID& leftStick, frc::GenericHID& rightStick);
	void Polar(double magnitude, double angle, double zRotation);

	void Cartesian(frc::GenericHID* leftStick, frc::GenericHID* rightStick, 
			double gyroAngle = 0.0);
	void Cartesian(frc::GenericHID& leftStick, frc::GenericHID& rightStick, 
			double gyroAngle = 0.0);
	void Cartesian(double ySpeed, double xSpeed, double zRotation,
			double gyroAngle = 0.0);
	void Cartesian(frc::Joystick& Stick,
			double gyroAngle = 0.0);

	void SetLeftRightMotorOutputs(double leftOutput, double rightOutput);
	void SetInvertedMotor(int side, bool isInverted);
	bool DriveOk();

 private:
	void InitDalekDrive();
	double squareInput(double v);
	void printFaults(int side, int faults);
	rev::CANSparkMax *m_leftMotor[NUM_MOTORS_PER_SIDE];
	rev::CANSparkMax *m_rightMotor[NUM_MOTORS_PER_SIDE];
    frc::SpeedControllerGroup *m_left;
    frc::SpeedControllerGroup *m_right;
	frc::DifferentialDrive *m_diffdrive;
	frc::MecanumDrive *m_mecanum;
	rev::CANEncoder *m_leftEncoder[NUM_MOTORS_PER_SIDE];
	rev::CANEncoder *m_rightEncoder[NUM_MOTORS_PER_SIDE];
	driveType m_type;
	bool m_needFree;
};
