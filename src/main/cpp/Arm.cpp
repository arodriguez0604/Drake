// for now, both motors will move into position at the same time... however this might need to change in order to keep in bounds and not hit walls in front of us 

#include "Arm.h"

Arm::Arm(int shoulderMotor, int elbowMotor, int turretMotor, int shoulderPot)
{
    m_shoulderMotor        = new CANSparkMax(shoulderMotor, CANSparkMax::MotorType::kBrushless);
    m_elbowMotor           = new WPI_TalonSRX(elbowMotor);
    m_turretMotor          = new WPI_TalonSRX(turretMotor);
    m_shoulderPot          = new AnalogPotentiometer(shoulderPot, 1.0, 0.0);
    m_shoulderMotorEncoder = new CANEncoder(*m_shoulderMotor);
    ArmInit();
    curX = 0;
    curY = 0;
    shoulderAngle = M_PI / 2;
}

Arm::Arm(CANSparkMax *shoulderMotor, WPI_TalonSRX *elbowMotor, WPI_TalonSRX *turretMotor, AnalogPotentiometer *shoulderPot)
{
    m_shoulderMotor        = shoulderMotor;
    m_elbowMotor           = elbowMotor;
    m_turretMotor          = turretMotor;
    m_shoulderPot          = shoulderPot;
    m_shoulderMotorEncoder = new CANEncoder(*m_shoulderMotor);
    ArmInit();
    curX = 0;
    curY = 0;
}

void
Arm::ArmInit()
{
    // needed to configure the talon to make sure things are ready for position mode
    m_elbowMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0);
    m_elbowMotor->SetSensorPhase(true);
    m_elbowMotor->Config_kF(0, 0.0, 0);
	m_elbowMotor->Config_kP(0, 0.3, 0);
	m_elbowMotor->Config_kI(0, 0.0, 0);
    m_elbowMotor->Config_kD(0, 0.2, 0);
}
float
Arm::DeadZone(float input, float range) {
    if (abs(input) < range) {
        return 0;
    } else {
        return input;
    }
}

void Arm::Tick(XboxController *xbox, POVButton *dPad[])
{
    m_turretMotor->Set(DeadZone(xbox->GetX(GenericHID::JoystickHand::kRightHand), .15) * .5);
    float x = 0;
    float y = 0;
    bool move = true;
    if (xbox->GetAButton()) {
        if (dPad[R]->Get()) {
            x = defaultX;
            y = rocketHatchLowHeight;
        } else if (dPad[T]->Get()) {
            x = cargoBallLength;
            y = cargoBallHeight;
        } else if (dPad[L]->Get()) {
            x = defaultX;
            y = cargoHatchHeight;
        } else if (dPad[B]->Get()) {
            x = defaultX;
            y = rocketBallLowHeight;
        } else {
            move = false;
        }
    } else if (xbox->GetBButton()) {
        if (dPad[R]->Get() || dPad[T]->Get() || dPad[L]->Get() || dPad[B]->Get()) {
            x = defaultX;
            y = ballLoadHeight;
        } else {
            x = ballPickUpX;
            y = ballPickUpY;
        }
    } else if (xbox->GetXButton()) {
        if (dPad[R]->Get()) {
            x = defaultX;
            y = rocketHatchMiddleHeight;
        } else if (dPad[B]->Get()) {
            x = defaultX;
            y = rocketBallMiddleHeight;
        } else {
            move = false;
        }
    } else if (xbox->GetYButton()) {
        if (dPad[R]->Get()) {
            x = defaultX;
            y = rocketHatchTopHeight;
        } else if (dPad[B]->Get()) {
            x = defaultX;
            y = rocketBallTopHeight;
        } else {
            move = false;
        }
    } else if (xbox->GetTriggerAxis(GenericHID::JoystickHand::kRightHand) > .1) {
        // FETAL POSITION
    } else {
        float xShift = DeadZone(xbox->GetY(GenericHID::JoystickHand::kRightHand), .4) * .5; // .5 is a guess... fix in testing
        float yShift = DeadZone(xbox->GetY(GenericHID::JoystickHand::kLeftHand), .4) * .5;  // same as above
        if (xShift == 0 && yShift == 0) {
            move = false;
        } else {
            x += xShift;
            y += yShift;
        }
    }
    if (move) {
        moveToPosition(x, y);
    }
    cout << m_shoulderPot->Get() << "\n";
    // if (xbox->GetAButton()) {
    //     shoulderAngle = M_PI / 2;
    // } else if (xbox->GetBButton()) {
    //     shoulderAngle = 2 * M_PI / 3;
    // }
    // SetMotors();
    // ha ha ha ha ha h a
}

void
Arm::moveToPosition(float x, float y)
{
    float ang1 = -1, ang2 = -1;
    if (FindArmAngles(x, y, &ang1, &ang2)) {
        shoulderAngle = ang1;
        elbowAngle = ang2;
        curX = x;
        curY = y;
        // std::cout << "MOVING TO: x = " << x << ", y = " << y << ", ang1 = " << ang1 << ", ang2 = " << ang2 << "\n";
    } else {
        // std::cout << "INVALID ANGLE: x = " << x << ", y = " << y << ", ang1 = " << ang1 << ", ang2 = " << ang2 << "\n";
    }
}

// the functions used to set the potentiometers need to change based on the robot
void
Arm::SetMotors() {
    // set the position the potentiometer should be at in the PID closed loop of the elbow Talon (second parameter is a voltage 0-3.3 i think)
    elbowAngle = M_PI / 2;
    m_elbowMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, -.553446 * elbowAngle + 2.38284);
    // set the shoulder speed
    // the shoulder elbow might me inverted
    // if (abs(m_shoulderPot->Get() - (shoulderAngle + 1.57331) / 6.0863) > .01) { // .1 is a placeholder for how close the motor can get at full power
    //     if (m_shoulderPot->Get() > (shoulderAngle + 1.57331) / 6.0863) {
    //         m_shoulderMotor->Set(-1); // too fast?
    //     } else {
    //         m_shoulderMotor->Set(1); // same
    //     }
    // } else {
    //     if (abs(m_shoulderPot->Get() - (shoulderAngle + 1.57331) / 6.0863) > .0005) { // .01 is a placeholder for how close to let the motor get without any extra adjustment
    //         if (m_shoulderPot->Get() > (shoulderAngle + 1.57331) / 6.0863) {
    //             m_shoulderMotor->Set(-.1); // this is a guess
    //         } else {
    //             m_shoulderMotor->Set(.1); // also a guess
    //         }
    //     } else {
    //         m_shoulderMotor->Set(0);
    //     }
    // }
    cout << "shoulderAngle: " << shoulderAngle << "\n" << m_shoulderPot->Get() << "\n" << (shoulderAngle + 1.57331) / 6.0863 << "\n";
}

// this function takes in the x distance from the target (starting from the edge of the drive train), and the y from the ground
bool
Arm::FindArmAngles(float x, float y, float *ang1, float *ang2)
{
	y -= armBaseHeight;
    // must make the x value vary based on where the turret is (for now i assume it is facing forward)
    x += armBaseFrontX - clawLength;
    float r = sqrt(x*x+y*y);
	*ang2 = acos((highArmLength * highArmLength + lowArmLength * lowArmLength - r * r) / (2 * highArmLength * lowArmLength));
	*ang1 = acos((lowArmLength * lowArmLength + r * r - highArmLength * highArmLength) / (2 * lowArmLength * r)) + atan(y / x);
    // HERE must find out whether or not the angles are allowed on the robot and return true or false accordingly
    return (*ang1 > 0 && *ang2 > 0);
}

// void
// Arm::FindArmMinMax(float base, float *elbowMin, float *elbowMax)
// {
// 	*elbowMin = .38*((150.0-base)/.8)+10.0;
// 	*elbowMax = .45*((150.0-base)/.8)+110.0-*elbowMin;
// }

void 
Arm::printVoltage(Joystick *bbb)
{
    //min: .156 max: .158
    SmartDashboard::PutNumber("Shoulder current", m_shoulderMotor->GetOutputCurrent());
    //min:.125 max:.375
    SmartDashboard::PutNumber("Elbow current", m_elbowMotor->GetOutputCurrent());
    //min:.125 max:.8
    SmartDashboard::PutNumber("Turret current", m_turretMotor->GetOutputCurrent());
    SmartDashboard::PutNumber("Elbow position", m_elbowMotor->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("Elbow close loop error",
        m_elbowMotor->GetClosedLoopError(0));
}
