#include "Lift.hpp"

LiftManager::LiftManager() {
stick = new frc::Joystick{ 0 };
xbox = new frc::XboxController{ 1 };

leftClimber = new WPI_TalonSRX(11);
rightClimber = new WPI_TalonSRX(9);
backClimber = new WPI_TalonSRX(10);

/*
leftClimber->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, K_PID_LOOP_IDX, K_TIMEOUT_MS);
leftClimber->SetSensorPhase(true);
leftClimber->ConfigAllowableClosedloopError(K_PID_LOOP_IDX, 0, K_TIMEOUT_MS);

leftClimber->Config_kP(K_PID_LOOP_IDX, 0.0, K_TIMEOUT_MS); 
leftClimber->Config_kI(K_PID_LOOP_IDX, 0.0, K_TIMEOUT_MS);
leftClimber->Config_kD(K_PID_LOOP_IDX, 0.0, K_TIMEOUT_MS);
leftClimber->Config_kF(K_PID_LOOP_IDX, 0.0, K_TIMEOUT_MS); */

verticalClimberSpeed = new double;
rightStick = new double;
leftStick = new double;

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
    *rightStick = xbox->GetRawAxis(3) * 0.35; //no more than 25%
    *leftStick = xbox->GetRawAxis(2) * 0.35; //no more than 25%

    if (*rightStick > *leftStick) {
        *verticalClimberSpeed = *rightStick;
    }
    else if (*leftStick > *rightStick) {
        *verticalClimberSpeed = *leftStick;
    }

    //*horz = xbox->GetRawAxis(5) * 0.2; 
    frc::SmartDashboard::PutNumber("liftPower", *verticalClimberSpeed);

    *leftDistance = (1.0 * -leftClimber->GetSensorCollection().GetQuadraturePosition() / 4096);
    *rightDistance = (1.0 * rightClimber->GetSensorCollection().GetQuadraturePosition() / 4096);
    *backDistance = (1.0 * backClimber->GetSensorCollection().GetQuadraturePosition() / 4096);

    frc::SmartDashboard::PutNumber("leftClimberEncoder", *leftDistance);
    frc::SmartDashboard::PutNumber("rightClimberEncoder", *rightDistance);
    frc::SmartDashboard::PutNumber("backClimberEncoder", *backDistance);

    if (*verticalClimberSpeed > 0) {
        if (*leftDistance > *rightDistance and *leftDistance > *backDistance) {
            *rightBoost = (*leftDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *backBoost = (*leftDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *leftBoost = 0;
        }
        else if (*rightDistance > *leftDistance and *rightDistance > *backDistance) {
            *leftBoost = (*rightDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *backBoost = (*rightDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *rightBoost = 0;
        }
        else if (*backDistance > *leftDistance and *backDistance > *leftDistance) {
            *leftBoost = (*backDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *rightBoost = (*backDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *backBoost = 0;
        }

        *leftPower = *leftBoost + *verticalClimberSpeed;
        *rightPower = *rightBoost + *verticalClimberSpeed;
        *backPower = *backBoost + *verticalClimberSpeed;
    }

    if (*verticalClimberSpeed < 0) {
        if (*leftDistance < *rightDistance and *leftDistance < *backDistance) {
            *rightBoost = (*leftDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *backBoost = (*leftDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *leftBoost = 0;
        }
        else if (*rightDistance < *leftDistance and *rightDistance < *backDistance) {
            *leftBoost = (*rightDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *backBoost = (*rightDistance - *backDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *rightBoost = 0;
        }
        else if (*backDistance < *leftDistance and *backDistance < *leftDistance) {
            *leftBoost = (*backDistance - *leftDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *rightBoost = (*backDistance - *rightDistance) / CLIMBER_SEPERATION_ROTATIONS;
            *backBoost = 0;
        }

        *leftPower = *leftBoost + *verticalClimberSpeed;
        *rightPower = *rightBoost + *verticalClimberSpeed;
        *backPower = *backBoost + *verticalClimberSpeed;
    }

    
    leftClimber->Set(-*leftPower); 
    rightClimber->Set(*rightPower);
    backClimber->Set(*backPower); 
 
  /*  leftClimber->Set(-*verticalClimberSpeed);
    rightClimber->Set(*verticalClimberSpeed);
    backClimber->Set(*verticalClimberSpeed); */

    frc::SmartDashboard::PutNumber("leftEnc", leftClimber->GetSensorCollection().GetQuadraturePosition());

//leftClimber->GetSensorCollection().IsFwdLimitSwitchClosed();
}