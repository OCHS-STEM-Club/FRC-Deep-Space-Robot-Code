#include "Manipulator.hpp" 

//Defines the namespace we'll be using, frc
using namespace frc;

ManipulatorManager::ManipulatorManager() {
    stick = new Joystick{ 0 };
    xbox = new XboxController{ 1 };

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

    //Initializes the variables used for the perimeterCheck() method
    startingAngle = new double;
    potDegrees = new double; 
    currentPotentiometerArmAngle = new double;
    *startingAngle = potentiometer->Get();
    calculatedPotentiometerMaximumArmLength = new double;

    calculatedEncoderArmAngle = new double;
    calculatedEncoderArmLength = new double;
    calculatedEncoderMaximumArmLength = new double;

    outOfFramePerimeterBool = new bool;

    armLatch = new bool;
    armToggle = new int; 
    *armToggle = 0;
    *armLatch = false;

    armMotor->SetNeutralMode(Brake);

    extendMotor->GetSensorCollection().SetQuadraturePosition(0, 10);
    armMotor->GetSensorCollection().SetQuadraturePosition(0, 10);
}

//Defines getters and setters for the arm variables

// double armDoubleVariableGetter(double armVariable) {
//     return armVariable;
// }

// void armDoubleVariableSetter(double armVariable, double value) {
//     armVariable = value;
// }

// bool armBoolVariableGetter(bool armVariable) {
//     return armVariable;
// }

// void armBoolVariableSetter(bool armVariable, bool value) {
//     armVariable = value;
// }

// //Defines the encoderUnitConverter method which converts encoder units into other units of choice
// double encoderUnitConverter(double slope, std::string motorType, double yIntercept) {
//     double convertedUnits;
//     //Checks the motorType string for the type of motor needed to convert
//     if(motorType.compare("Arm Motor")) {
//         convertedUnits = ((ENCODER_UNIT_TO_DEGREE_SLOPE * armMotor->GetSensorCollection().GetQuadraturePosition()) + ENCODER_UNIT_ARM_ANGLE_Y_INTERCEPT);
//     }
    
//     else if(motorType.compare("Extend Motor")) {
//         convertedUnits = ((ENCODER_UNIT_TO_INCH_SLOPE * extendMotor->GetSensorCollection().GetQuadraturePosition()) + ENCODER_UNIT_ARM_LENGTH_Y_INTERCEPT);
//     }

//     return convertedUnits;
// }

//Defines the maximumArmLength method which calculates the maximum arm length without breaking the frame perimeter
double maximumArmLengthCalculator(double armAngle) {
    double maximumArmLength;
    double cosArmAngle;

    //Checks whether or not armAngle is at the ABSOLUTE_VERTICAL_ARM_ANGLE
    if(armAngle == ABSOLUTE_VERTICAL_ARM_ANGLE) {
        //If true, set armAngle to the absolute max
        maximumArmLength = ABSOLUTE_MAXIMUM_ARM_LENGTH_VERTICAL;
    }
    
    else {
        //Otherwise, calculate the maximumArmLength by hand
        cosArmAngle = cos(armAngle * DEGREE_TO_RADIAN_RATIO);
        maximumArmLength = abs(MAXIMUM_ARM_LENGTH_PARALLEL/cosArmAngle);
        // maximumArmLength = armAngle * DEGREE_TO_RADIAN_RATIO;
    }
    
    return maximumArmLength;
}

//Defines the outOfFramePerimeter method which checks whether or not the arm is breaking the frame perimeter, true if it passes, false if it fails
bool outOfFramePerimeterCheck(double calculatedArmLength, double calculatedMaximumArmLength1, double calculatedMaximumArmLength2) {
    //Defines the outOfFramePerimeter bool as a representation of whether or not the arm is past its maximum length, and by extension, the frame perimeter
    bool outOfFramePerimeter = ((calculatedArmLength > calculatedMaximumArmLength1) || (calculatedArmLength > calculatedMaximumArmLength2));
    
    //Checks outOfFramePerimeter's parity
    if(!outOfFramePerimeter) {
        //If true, return false
        return false;
    }
    
    //Otherwise, return true
    else {        
        return true;
    }
}

//Defines the perimeterCheck method which retracts the arm if it is outside of the frame perimeter
void ManipulatorManager::perimeterCheck() {
   
    //Sets the approximate arm angle and length based on the encoder and potentiometer values
    
    // armDoubleVariableSetter(*calculatedEncoderArmAngle, ((ENCODER_UNIT_TO_DEGREE_SLOPE * armMotor->GetSensorCollection().GetQuadraturePosition()) + ENCODER_UNIT_ARM_ANGLE_Y_INTERCEPT));
    // armDoubleVariableSetter(*calculatedEncoderArmLength, ((ENCODER_UNIT_TO_INCH_SLOPE * extendMotor->GetSensorCollection().GetQuadraturePosition()) + ENCODER_UNIT_ARM_LENGTH_Y_INTERCEPT));
    // armDoubleVariableSetter(*calculatedEncoderMaximumArmLength, maximumArmLengthCalculator(*calculatedEncoderArmAngle));
    // armDoubleVariableSetter(*calculatedPotentiometerMaximumArmLength, maximumArmLengthCalculator(*currentPotentiometerArmAngle));

    *calculatedEncoderArmAngle = ((ENCODER_UNIT_TO_DEGREE_SLOPE * armMotor->GetSensorCollection().GetQuadraturePosition()) + ENCODER_UNIT_ARM_ANGLE_Y_INTERCEPT);
    *calculatedEncoderArmLength = ((ENCODER_UNIT_TO_INCH_SLOPE * extendMotor->GetSensorCollection().GetQuadraturePosition()) + ENCODER_UNIT_ARM_LENGTH_Y_INTERCEPT);
    *calculatedEncoderMaximumArmLength = maximumArmLengthCalculator(*calculatedEncoderArmAngle);
    *calculatedPotentiometerMaximumArmLength = maximumArmLengthCalculator(*currentPotentiometerArmAngle);

    //Sets the outOfFramePerimeterBool to the value returned by the checking function
    // armBoolVariableSetter(*outOfFramePerimeter, outOfFramePerimeterCheck(*calculatedEncoderArmLength, *calculatedEncoderMaximumArmLength, *calculatedPotentiometerMaximumArmLength));
    *outOfFramePerimeterBool = outOfFramePerimeterCheck(*calculatedEncoderArmLength, *calculatedEncoderMaximumArmLength, *calculatedPotentiometerMaximumArmLength);

    //Puts the numerous variables in use in the method on the Smart Dashboard
    frc::SmartDashboard::PutBoolean("Out of Frame Perimeter", *outOfFramePerimeterBool);
    frc::SmartDashboard::PutNumber("Calculated Encoder Arm Angle", *calculatedEncoderArmAngle);
    frc::SmartDashboard::PutNumber("Calculated Encoder Arm Length", *calculatedEncoderArmLength);
    frc::SmartDashboard::PutNumber("Calculated Encoder Maximum Arm Length", *calculatedEncoderMaximumArmLength);
    frc::SmartDashboard::PutNumber("Calculated Potentiometer Maximum Arm Length", *calculatedPotentiometerMaximumArmLength);
} 

void ManipulatorManager::manipulate() {
    *armSpeed = xbox->GetRawAxis(5) * 0.45;
    //Checks the value of *outOfFramePerimeterBool
    if(!*outOfFramePerimeterBool) {
        //If false, get the extendSpeed value from the xbox left stick
        *extendSpeed = -xbox->GetRawAxis(1) * 0.7;   //negative is out
    }

    else {
        //Otherwise, retract the arm
        *extendSpeed = FRAME_PERIMETER_ARM_RETRACTION_SPEED;
    }
    armMotor->Set(*armSpeed);
    extendMotor->Set(*extendSpeed);

    frc::SmartDashboard::PutBoolean("hall effect", !hallEffect->Get());

    *potDegrees = potentiometer->Get();
    *currentPotentiometerArmAngle = -(*potDegrees - *startingAngle) + STARTING_ARM_ANGLE;

    frc::SmartDashboard::PutNumber("potentiometer angle", *potDegrees);
    frc::SmartDashboard::PutNumber("Calculated Potentiometer Arm Angle", *currentPotentiometerArmAngle);

    if (xbox->GetRawButton(1)) {
        handMotor->Set(0.4);
    }
    else if (xbox->GetRawButton(2)) {
        handMotor->Set(-0.4);
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
