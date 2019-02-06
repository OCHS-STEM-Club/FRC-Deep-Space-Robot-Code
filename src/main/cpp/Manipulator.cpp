#include "Manipulator.hpp" 

ManipulatorManager::ManipulatorManager() {
    stick = new frc::Joystick{ 0 };
    xbox = new frc::XboxController{ 1 };

    armMotor = new WPI_TalonSRX(8);
    extenderMotor = new WPI_TalonSRX(7);

    armSpeed = new double;

    armMotor->SetNeutralMode(Brake);
}

void ManipulatorManager::manipulate() {
    *armSpeed = xbox->GetRawAxis(5) * 0.4;

    armMotor->Set(*armSpeed);
}