#pragma once

#include <frc/Joystick.h>
#include <ctre/Phoenix.h> 
#include <Drive/DifferentialDrive.h>

class DriveManager {
    private:
    Joystick *stick; 
    WPI_TalonSRX *driveMotorLeft; 
    WPI_TalonSRX *driveMotorRight; 

    DifferentialDrive *tankDrive;

    double *xStickValue;
    double *yStickValue;

    public:
    DriveManager();
    void driveTrain(); 
};