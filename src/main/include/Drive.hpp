#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
//#include <frc/MotorSafety.h>
//#include <frc/RobotDriveBase.h>
//#include <frc/DifferentialDrive.h>
#include <ctre/Phoenix.h>

class DriveManager {
    private:
    frc::Joystick *stick; 
    WPI_TalonSRX *driveMotorLeft; 
    WPI_TalonSRX *driveMotorRight; 

    frc::DifferentialDrive *tankDrive;

    double *xStickValue;
    double *yStickValue;

    public:
    DriveManager();
    void driveTrain(); 
};