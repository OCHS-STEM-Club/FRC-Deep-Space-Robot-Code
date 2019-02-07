#include "Lift.hpp"

LiftManager::LiftManager() {
stick = new frc::Joystick{ 0 };
xbox = new frc::XboxController{ 1 };

leftClimber = new WPI_TalonSRX(11);
rightClimber = new WPI_TalonSRX(9);
backClimber = new WPI_TalonSRX(10);

verticalClimberSpeed = new double;

leftDistance = new double;
rightDistance = new double;
backDistance = new double;

leftBoost = new double; 
rightBoost = new double;
backBoost = new double;

leftPower = new double;
rightPower = new double;
backPower = new double;


leftClimber->SetNeutralMode(Brake);
rightClimber->SetNeutralMode(Brake);
backClimber->SetNeutralMode(Brake);

leftClimber->GetSensorCollection().SetQuadraturePosition(0,10);
rightClimber->GetSensorCollection().SetQuadraturePosition(0,10);
backClimber->GetSensorCollection().SetQuadraturePosition(0,10);
}

void LiftManager::Lift() {
    *verticalClimberSpeed = xbox->GetRawAxis(1) * 0.25; //no more than 25%
    //*horz = xbox->GetRawAxis(5) * 0.2; 

    *leftDistance = (1.0 * leftClimber->GetSensorCollection().GetQuadraturePosition() / 4096);
    *rightDistance = (1.0 * rightClimber->GetSensorCollection().GetQuadraturePosition() / 4096);
    *backDistance = (1.0 * backClimber->GetSensorCollection().GetQuadraturePosition() / 4096);

    frc::SmartDashboard::PutNumber("leftClimberEncoder", *leftDistance);
    frc::SmartDashboard::PutNumber("rightClimberEncoder", *rightDistance);
    frc::SmartDashboard::PutNumber("backClimberEncoder", *backDistance);

    if (*leftDistance > *rightDistance and *leftDistance > *backDistance) {
        *rightBoost = (*leftDistance - *rightDistance) / 4;
        *backBoost = (*leftDistance - *backDistance) / 4;
        *leftBoost = 0;
    }
    else if (*rightDistance > *leftDistance and *rightDistance > *backDistance) {
        *leftBoost = (*rightDistance - *leftDistance) / 4;
        *backBoost = (*rightDistance - *backDistance) / 4;
        *rightBoost = 0;
    }
    else if (*backDistance > *leftDistance and *backDistance > *leftDistance) {
        *leftBoost = (*backDistance - *leftDistance) / 4;
        *rightBoost = (*backDistance - *rightDistance) / 4;
        backBoost = 0;
    }

    *leftPower = *leftBoost + *verticalClimberSpeed;
    *rightPower = *rightBoost + *verticalClimberSpeed;
    *backPower = *backBoost + *verticalClimberSpeed;
 
    leftClimber->Set(-*leftPower); 
    rightClimber->Set(*rightPower);
    backClimber->Set(*backPower);
 

//leftClimber->GetSensorCollection().IsFwdLimitSwitchClosed();
}