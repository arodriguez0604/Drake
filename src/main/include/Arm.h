#pragma once

// claw servo position must be public or have a public get function

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/buttons/POVButton.h>
#include <frc/XboxController.h>

#include <frc/Joystick.h>

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashBoard.h"

using namespace rev;
using namespace std;

// in millimeters (check these)
#define armBaseHeight             342.9
#define armBaseFrontX             387.35
#define armBaseSideX              342.9
#define lowArmLength              812.8
#define highArmLength             1079.5
#define defaultX                  509.6                    // i made this up
#define cargoHatchHeight          298          // 1
#define cargoBallHeight           1003         // 2 need to increase height
#define cargoBallLength           508                    // i made this up
#define rocketHatchLowHeight      298  // check   3
#define rocketBallLowHeight       679.45 // 4  It WORKS 
#define rocketHatchMiddleHeight   1047.75 // 5   IT WORKS 
#define rocketBallMiddleHeight    1422// 6 It WORKS
#define rocketHatchTopHeight      1740// 7  IT WORKS
#define rocketBallTopHeight       2096// 8  IT WORKS
#define ballPickUpX               279 
#define rocketTopHeightX          76 // IT WORKS
#define ballPickUpY               165 
#define discLoadHeight            298// only if we are hooking it
#define ballLoadHeight            1101 //unknown    11  
#define ballLoadX                 368    //WRITE THIS ONE
#define rocketTopHeightBallX      76

#define TURRET_LEFT               0 // find this 
#define TURRET_RIGHT              0 // find this 
#define TURRET_CENTER             0 // find this 
#define TURRET_NONE               0
#define turretOffset              30.48

enum POVButtons {R, T, L, B};


using namespace frc;

class Arm {
  public:
    float turretPosition, shoulderAngle, elbowAngle, curX, curY;
    bool fetalPosition;

    Arm(int shoulderMotor, int elbowMotor, int turretMotor, int shoulderPot);
    Arm(CANSparkMax *shoulderMotor, WPI_TalonSRX *elbowMotor, 
          WPI_TalonSRX *turretMotor, AnalogPotentiometer *shoulderPot);

    void Tick(XboxController *xbox, POVButton *dPad[4]);
    void moveToPosition(float x, float y);
    void printInfo();
  
  private:
    CANSparkMax *m_shoulderMotor;
    CANEncoder *m_shoulderMotorEncoder;
    WPI_TalonSRX *m_elbowMotor, *m_turretMotor;
    AnalogPotentiometer *m_shoulderPot;
    PIDController *m_shoulderController;

    void SetMotors();
    void ArmInit();
    bool validElbowPosition(double pos);
    double computeElbowPosition(double angle);
    bool validShoulderPosition(double pos);
    double computeShoulderPosition(double angle);
    bool FindArmAngles(float x, float y, float *ang1, float *ang2);
    // void FindArmMinMax(float base, float *elbowMin, float *elbowMax);
    float DeadZone(float input, float range);
};

