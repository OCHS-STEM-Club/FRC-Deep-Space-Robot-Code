#pragma once

#include <iostream>
#include <string>
#include <cmath>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

//Defines the numerous constants used for the perimeterCheck() method in Manipulator src
#define STARTING_ARM_ANGLE 58
#define ENCODER_UNIT_TO_DEGREE_SLOPE -0.02154
#define ENCODER_UNIT_ARM_ANGLE_Y_INTERCEPT 58
#define ENCODER_UNIT_TO_INCH_SLOPE 0.001824
#define ENCODER_UNIT_ARM_LENGTH_Y_INTERCEPT -0.1259
#define ABSOLUTE_VERTICAL_ARM_ANGLE 90
#define ABSOLUTE_MAXIMUM_ARM_LENGTH_VERTICAL 67
#define MAXIMUM_ARM_LENGTH_PARALLEL 48
#define FRAME_PERIMETER_ARM_RETRACTION_SPEED 0.5
#define DEGREE_TO_RADIAN_RATIO M_PI/180.0

class ManipulatorManager {
    private:
    frc::Joystick *stick; 
    frc::XboxController *xbox;

    frc::AnalogPotentiometer *potentiometer;
    frc::DigitalInput *hallEffect;
    frc::DigitalInput *notExtendedLimit;

    WPI_TalonSRX *armMotor;
    WPI_TalonSRX *extendMotor;
    WPI_TalonSRX *handMotor;

    frc::AnalogInput *ultra;

    //double *pidControl;
    //frc::PIDController *pid;

    double *armSpeed;
    double *extendSpeed;

    //Defines the numerous variables used for the calculation of the perimeterCheck() method in the Manipulator src
    double *startingAngle;
    double *potDegrees;

    double *currentPotentiometerArmAngle;
    double *calculatedPotentiometerMaximumArmLength;

    double *calculatedEncoderArmAngle;
    double *calculatedEncoderArmLength;
    double *calculatedEncoderMaximumArmLength;

    bool *outOfFramePerimeterBool;

    bool *armLatch;
    int *armToggle;

    //Defines the methods called within the Manipulator src
    public:
    ManipulatorManager();
    void manipulate();
    void perimeterCheck();
};