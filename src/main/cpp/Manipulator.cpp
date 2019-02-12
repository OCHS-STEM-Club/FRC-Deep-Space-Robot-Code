#include "Manipulator.hpp" 

ManipulatorManager::ManipulatorManager() {
    stick = new frc::Joystick{ 0 };
    xbox = new frc::XboxController{ 1 };

    potentiometer = new frc::AnalogPotentiometer(3, 3600, 0.0);
    hallEffect = new frc::DigitalInput(0);

    armMotor = new WPI_TalonSRX(8);
    extendMotor = new WPI_TalonSRX(7);
    handMotor = new WPI_TalonSRX(6);

    //pidControl = new double;
    //pid = new frc::PIDController(0 ,0, 0, &potentiometer, armMotor);

    armSpeed = new double;
    extendSpeed = new double;

    startingAngle = new double;
    potDegrees = new double; 
    caculatedAngle = new double;
    *startingAngle = potentiometer->Get();

    armMotor->SetNeutralMode(Brake);
}

void ManipulatorManager::manipulate() {
    *armSpeed = xbox->GetRawAxis(5) * 0.45;
    *extendSpeed = -xbox->GetRawAxis(1) * 0.7;

    armMotor->Set(*armSpeed);
    extendMotor->Set(*extendSpeed);

    frc::SmartDashboard::PutBoolean("hall effect", -hallEffect->Get());

    *potDegrees = potentiometer->Get();
    *caculatedAngle = *potDegrees - *startingAngle;

    frc::SmartDashboard::PutNumber("potentiometer angle", *potDegrees);
    frc::SmartDashboard::PutNumber("caculated angle", *caculatedAngle);

    if (xbox->GetRawButton(1)) {
        handMotor->Set(0.2);
    }
    else if (xbox->GetRawButton(2)) {
        handMotor->Set(-0.2);
    }
    else {
        handMotor->Set(0);
    }
}