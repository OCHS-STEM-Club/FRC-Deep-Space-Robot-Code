#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

#define STARTING_ARM_ANGLE 65


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

    //double *pidControl;
    //frc::PIDController *pid;

    double *armSpeed;
    double *extendSpeed;

    double *startingAngle;
    double *potDegrees;
    double *caculatedAngle;

    bool *armLatch;
    int *armToggle;


    public:
    ManipulatorManager();
    void manipulate();
};