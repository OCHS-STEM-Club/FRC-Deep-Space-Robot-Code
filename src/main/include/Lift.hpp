#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

#define CLIMBER_SEPERATION_ROTATIONS 1.5

#define K_TIMEOUT_MS 10
#define K_PID_LOOP_IDX 0

class LiftManager {
    private:
    frc::Joystick *stick; 
    frc::XboxController *xbox;

    WPI_TalonSRX *leftClimber; 
    WPI_TalonSRX *rightClimber; 
    WPI_TalonSRX *backClimber; 
    
    double *verticalClimberSpeed;
    double *rightStick;
    double *leftStick;
    //double *horz;

    double *leftDistance;
    double *rightDistance;
    double *backDistance;

    double *leftBoost;
    double *rightBoost;
    double *backBoost;

    double *leftPower;
    double *rightPower;
    double *backPower;

    public:
    LiftManager();
    void Lift();
};