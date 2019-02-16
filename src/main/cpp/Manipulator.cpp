#include "Manipulator.hpp" 

ManipulatorManager::ManipulatorManager() {
    stick = new frc::Joystick{ 0 };
    xbox = new frc::XboxController{ 1 };

    potentiometer = new frc::AnalogPotentiometer(3, 3600, 0.0);
    hallEffect = new frc::DigitalInput(0);
    notExtendedLimit = new frc::DigitalInput(1);

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

    armLatch = new bool;
    armToggle = new int; 
    *armToggle = 0;
    *armLatch = false;

    armMotor->SetNeutralMode(Brake);

    extendMotor->GetSensorCollection().SetQuadraturePosition(0, 10);
    armMotor->GetSensorCollection().SetQuadraturePosition(0, 10);
}

void ManipulatorManager::manipulate() {
    *armSpeed = xbox->GetRawAxis(5) * 0.45;
    *extendSpeed = -xbox->GetRawAxis(1) * 0.7;   //negitive is out

    armMotor->Set(*armSpeed);
    extendMotor->Set(*extendSpeed);

    frc::SmartDashboard::PutBoolean("hall effect", !hallEffect->Get());

    *potDegrees = potentiometer->Get();
    *caculatedAngle = *potDegrees - *startingAngle + STARTING_ARM_ANGLE;

    frc::SmartDashboard::PutNumber("potentiometer angle", *potDegrees);
    frc::SmartDashboard::PutNumber("caculated angle", *caculatedAngle);

    if (xbox->GetRawButton(1)) {
        handMotor->Set(0.25);
    }
    else if (xbox->GetRawButton(2)) {
        handMotor->Set(-0.25);
    }
    else {
        handMotor->Set(0);
    }
    frc::SmartDashboard::PutNumber("hand current", handMotor->GetOutputCurrent());

    frc::SmartDashboard::PutNumber("extend position", extendMotor->GetSensorCollection().GetQuadraturePosition());
    frc::SmartDashboard::PutNumber("arm position", armMotor->GetSensorCollection().GetQuadraturePosition());


    if (!hallEffect->Get() and !*armLatch) {
        *armLatch = true;

        if (*extendSpeed > 0) {
            *armToggle = *armToggle + 1;
        }
        else if (*extendSpeed < 0) {
            *armToggle = *armToggle - 1;
        }
    } 
    else if (hallEffect->Get() and *armLatch) {
        *armLatch = false;
    }
    frc::SmartDashboard::PutNumber("arm hall position", *armToggle);

    frc::SmartDashboard::PutBoolean("test limit", notExtendedLimit->Get());
    if (notExtendedLimit->Get()) {
        extendMotor->GetSensorCollection().SetQuadraturePosition(0, 10);
        //xbox->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0.5);
    }
}