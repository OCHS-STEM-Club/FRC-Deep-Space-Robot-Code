#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <AHRS.h> 
#include <rev/CANSparkMax.h>

class DriveManager {
    private:
    frc::Joystick *stick; 
    //WPI_TalonSRX *driveMotorFrontLeft; 
    //WPI_TalonSRX *driveMotorFrontRight; 
    //WPI_TalonSRX *driveMotorBackLeft; 
    //WPI_TalonSRX *driveMotorBackRight; 

    rev::CANSparkMax *driveMotorFrontLeft;
    rev::CANSparkMax *driveMotorFrontRight;
    rev::CANSparkMax *driveMotorBackLeft;
    rev::CANSparkMax *driveMotorBackRight; 

    //encoders for CANSparkMax
    rev::CANEncoder *encFrontLeft;
    rev::CANEncoder *encFrontRight;
    rev::CANEncoder *encBackLeft; 
    rev::CANEncoder *encBackRight; 

    frc::MecanumDrive *mecanumDrive;

    AHRS *ahrs; 

    double *xStickValue;
    double *yStickValue;
    double *zStickValue;

    double *driveGyro; 
    double *gyro;

    bool *driveToggle; 
    bool *driveLatch; 

    bool *idleModeToggle;
    bool *idleModeLatch;

    public:
    DriveManager();
    void driveTrain(); 
    void control(double turn, double strafe, double drive , bool brake); 
};