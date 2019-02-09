#include "Manipulator.hpp" 

ManipulatorManager::ManipulatorManager() {
    stick = new frc::Joystick{ 0 };
    xbox = new frc::XboxController{ 1 };

    armMotor = new WPI_TalonSRX(8);
    extendMotor = new WPI_TalonSRX(7);

    //pidControl = new double;
    //pid = new frc::PIDController(0 ,0, 0, armMotor->GetSensorCollection().GetQuadraturePosition(), pidControl, 0.5);

    armSpeed = new double;
    extendSpeed = new double;

    armMotor->SetNeutralMode(Brake);
}

void ManipulatorManager::manipulate() {
    *armSpeed = xbox->GetRawAxis(5) * 0.3;
    *extendSpeed = xbox->GetRawAxis(1) * 0.6;

    armMotor->Set(*armSpeed);
    extendMotor->Set(*extendSpeed);
}