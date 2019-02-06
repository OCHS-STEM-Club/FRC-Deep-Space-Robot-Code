#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

class LiftManager {
    private:
    frc::Joystick *stick; 
    frc::XboxController *xbox;

    WPI_TalonSRX *leftClimber; 
    WPI_TalonSRX *rightClimber; 
    WPI_TalonSRX *backClimber; 
    //WPI_TalonSRX *leftHorz;
    //WPI_TalonSRX *rightHorz;
    
    double *verticalClimberSpeed;
    //double *horz;

    public:
    LiftManager();
    void Lift();
};