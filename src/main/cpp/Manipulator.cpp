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

    //Initializes the variables used for the perimeterCheck() method
    startingAngle = new double;
    potDegrees = new double; 
    calculatedPotentiometerArmAngle = new double;
    *startingAngle = potentiometer->Get();

    calculatedPotentiometerArmAngle = new double;
    calculatedPotentiometerMaximumArmLength = new double;

    calculatedEncoderArmAngle = new double;
    calculatedEncoderArmLength = new double;
    calculatedEncoderMaximumArmLength = new double;

    outOfFramePerimeter = new bool;

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
    *calculatedPotentiometerArmAngle = *potDegrees - *startingAngle + STARTING_ARM_ANGLE;

    frc::SmartDashboard::PutNumber("potentiometer angle", *potDegrees);
    frc::SmartDashboard::PutNumber("calculated angle", *calculatedPotentiometerArmAngle);

    if (xbox->GetRawButton(1)) {
        handMotor->Set(0.2);
    }
    else if (xbox->GetRawButton(2)) {
        handMotor->Set(-0.2);
    }
    else {
        handMotor->Set(0);
    }

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
}

//Defines the perimeterCheck method which retracts the arm if it is outside of the frame perimeter
void ManipulatorManager::perimeterCheck() {
    //Calculates the approximate arm angle and length based on the encoder values
    *calculatedEncoderArmAngle = -(ENCODER_UNIT_TO_DEGREE_RATIO) + ENCODER_UNIT_ARM_ANGLE_Y_INTERCEPT;
    *calculatedEncoderArmLength = ENCODER_UNIT_TO_INCH_RATIO + ENCODER_UNIT_ARM_LENGTH_Y_INTERCEPT;
    
    //Checks if either of the calculated angles are at the ABSOLUTE_VERTICAL_ARM_ANGLE
    if(ABSOLUTE_VERTICAL_ARM_ANGLE == (*calculatedEncoderArmAngle || *calculatedPotentiometerArmAngle)) {
        //If true, it sets the calculated maximum arm length to ABSOLUTE_MAXIMUM_ARM_LENGTH_VERTICAL
        *calculatedPotentiometerMaximumArmLength = ABSOLUTE_MAXIMUM_ARM_LENGTH_VERTICAL;
        *calculatedEncoderMaximumArmLength = ABSOLUTE_MAXIMUM_ARM_LENGTH_VERTICAL;
    }

    //Otherwise, it calculates the maximum arm length manually
    else {
        *calculatedPotentiometerMaximumArmLength = double(abs(MAXIMUM_ARM_LENGTH_PARALLEL/cos(*calculatedPotentiometerArmAngle)));
        *calculatedEncoderMaximumArmLength = double(abs(MAXIMUM_ARM_LENGTH_PARALLEL/cos(*calculatedEncoderArmAngle)));
    }

    //Defines the outOfFramePerimeter bool as a representation of whether or not the arm is past its maximum length, and by extension, the frame perimeter
    *outOfFramePerimeter = *calculatedEncoderArmLength > (*calculatedEncoderMaximumArmLength || *calculatedPotentiometerMaximumArmLength);
    //If the arm is past its calculated maximum length, retract the arm until it isn't
    while(*outOfFramePerimeter) {
        extendMotor->Set(-(*extendSpeed));
    }

    //Puts the numerous variables in use on the Smart Dashboard
    frc::SmartDashboard::PutBoolean("Out of Frame Perimeter", *outOfFramePerimeter);
    frc::SmartDashboard::PutNumber("Calculated Encoder Arm Angle", *calculatedEncoderArmAngle);
    frc::SmartDashboard::PutNumber("Calculated Potentiometer Arm Angle", *calculatedPotentiometerArmAngle);
    frc::SmartDashboard::PutNumber("Calculated Encoder Arm Length", *calculatedEncoderArmLength);
    frc::SmartDashboard::PutNumber("Calculated Encoder Maximum Arm Length", *calculatedEncoderMaximumArmLength);
    frc::SmartDashboard::PutNumber("Calculated Potentiometer Maximum Arm Length", *calculatedPotentiometerMaximumArmLength);
} 