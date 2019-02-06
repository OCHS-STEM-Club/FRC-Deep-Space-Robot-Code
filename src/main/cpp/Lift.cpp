#include "Lift.hpp"

LiftManager::LiftManager() {
stick = new frc::Joystick{ 0 };
xbox = new frc::XboxController{ 1 };

leftClimber = new WPI_TalonSRX(11);
rightClimber = new WPI_TalonSRX(9);
backClimber = new WPI_TalonSRX(10);
//leftHorz = new WPI_TalonSRX(8);
//rightHorz = new WPI_TalonSRX(9);

verticalClimberSpeed = new double;

leftClimber->SetNeutralMode(Brake);
rightClimber->SetNeutralMode(Brake);
backClimber->SetNeutralMode(Brake);

//leftClimber->Follow(*rightClimber);
}

void LiftManager::Lift() {
    *verticalClimberSpeed = xbox->GetRawAxis(1); //no more than 25%
    //*horz = xbox->GetRawAxis(5) * 0.2; 

    leftClimber->Set(-*verticalClimberSpeed * 0.3); //is a follower
    rightClimber->Set(*verticalClimberSpeed * 0.25);
    backClimber->Set(*verticalClimberSpeed * 0.24);

   // leftHorz->Set(*horz);
   // rightHorz->Set(*horz);
}