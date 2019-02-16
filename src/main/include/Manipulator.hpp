#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

//Defines the numerous constants used for the perimeterCheck() method in Manipulator src
#define STARTING_ARM_ANGLE 65
#define ENCODER_UNIT_TO_DEGREE_RATIO 25.7077
#define ENCODER_UNIT_ARM_ANGLE_Y_INTERCEPT 2357
#define ENCODER_UNIT_TO_INCH_RATIO 548.1416
#define ENCODER_UNIT_ARM_LENGTH_Y_INTERCEPT 69
#define ABSOLUTE_VERTICAL_ARM_ANGLE 90
#define ABSOLUTE_MAXIMUM_ARM_LENGTH_VERTICAL 67
#define MAXIMUM_ARM_LENGTH_PARALLEL 48

class ManipulatorManager {
    private:
    frc::Joystick *stick; 
    frc::XboxController *xbox;

    frc::AnalogPotentiometer *potentiometer;
    frc::DigitalInput *hallEffect;

    WPI_TalonSRX *armMotor;
    WPI_TalonSRX *extendMotor;
    WPI_TalonSRX *handMotor;

    //double *pidControl;
    //frc::PIDController *pid;

    double *armSpeed;
    double *extendSpeed;

    //Defines the numerous variables used for the calculation of the perimeterCheck() method in the Manipulator src
    double *startingAngle;
    double *potDegrees;

    double *calculatedPotentiometerArmAngle;
    double *calculatedPotentiometerMaximumArmLength;

    double *calculatedEncoderArmAngle;
    double *calculatedEncoderArmLength;
    double *calculatedEncoderMaximumArmLength;

    bool *outOfFramePerimeter;

    bool *armLatch;
    int *armToggle;

    //Defines the methods called within the Manipulator src
    public:
    ManipulatorManager();
    void manipulate();
    void perimeterCheck();
};