#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

#define STARTING_ARM_ANGLE 0


class ManipulatorManager {
    private:
    frc::Joystick *stick; 
    frc::XboxController *xbox;

    frc::AnalogPotentiometer *potentiometer;
    frc::DigitalInput *hallEffect;

    WPI_TalonSRX *armMotor; 
    WPI_TalonSRX *extendMotor;

    //double *pidControl;
    //frc::PIDController *pid;

    double *armSpeed;
    double *extendSpeed;

    double *startingAngle;
    double *potDegrees;
    double *caculatedAngle;

    public:
    ManipulatorManager();
    void manipulate();
};