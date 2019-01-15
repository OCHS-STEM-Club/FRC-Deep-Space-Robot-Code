#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <AHRS.h> 

class DriveManager {
    private:
    frc::Joystick *stick; 
    WPI_TalonSRX *driveMotorFrontLeft; 
    WPI_TalonSRX *driveMotorFrontRight; 
    WPI_TalonSRX *driveMotorBackLeft; 
    WPI_TalonSRX *driveMotorBackRight; 

    frc::MecanumDrive *mecanumDrive;

    AHRS *ahrs; 

    double *xStickValue;
    double *yStickValue;
    double *zStickValue;

    double *driveGyro; 
    double *gyro;

    bool *driveToggle; 
    bool *driveLatch; 

    public:
    DriveManager();
    void driveTrain(); 
};