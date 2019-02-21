#include "Claw.h"

Claw::Claw(int clawMotor, int clawServo) {
    m_clawMotor = new WPI_TalonSRX(clawMotor);
    m_clawServo = new Servo(clawServo);
}

Claw::Claw(WPI_TalonSRX *clawMotor, Servo *clawServo) {
    m_clawMotor = clawMotor;
    m_clawServo = clawServo;
}

void Claw::Tick(XboxController *xbox) {
    if (xbox->GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand) > .1) {
        OpenServo();
    } else {
        CloseServo();
    }
    if (xbox->GetBumper(frc::GenericHID::JoystickHand::kLeftHand)) {
        RetrieveBall();
    } else if (xbox->GetBumper(frc::GenericHID::JoystickHand::kRightHand)) {
        EjectBall();
    } else {
        m_clawMotor->Set(0);
    }
}

void
Claw::OpenServo() {
    m_clawServo->SetAngle(servoOpen);
}

void
Claw::CloseServo() {
    m_clawServo->SetAngle(servoClosed);
}

void
Claw::RetrieveBall() {
    m_clawMotor->Set(1);
}

void
Claw::EjectBall() {
    m_clawMotor->Set(-1);
}

void 
Claw::printVoltage()
{
    //min: .25 max:.8
    SmartDashboard::PutNumber("CLaw Motor current", m_clawMotor->GetOutputCurrent());
    //frc::SmartDashboard::PutNumber("Servo current", m_clawServo->GetOutputCurrent());
}