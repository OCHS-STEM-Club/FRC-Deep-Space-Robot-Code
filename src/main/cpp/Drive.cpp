#include "Drive.hpp"

DriveManager::DriveManager () {
    stick = new frc::Joystick{ 0 };

    driveMotorFrontLeft = new WPI_TalonSRX(1);
    driveMotorFrontRight = new WPI_TalonSRX(2);
    driveMotorBackLeft = new WPI_TalonSRX(3);
    driveMotorBackRight = new WPI_TalonSRX(4);

    mecanumDrive = new frc::MecanumDrive(*driveMotorFrontLeft, *driveMotorBackLeft, *driveMotorFrontRight, *driveMotorBackRight);

    try {
        ahrs = new AHRS(SPI::Port::kMXP);
    }
    catch(std::exception ex) {
        std::string err_string = "Error initalizing naxX-MXP"; 
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }
    ahrs->Reset(); 

    xStickValue = new double; 
    yStickValue = new double; 
    zStickValue = new double; 
    
    driveGyro = new double;
    gyro = new double; 

    driveToggle = new bool;  
    driveLatch = new bool; 
    *driveToggle = false;
    *driveLatch = false;
}

void DriveManager::driveTrain() {
    *xStickValue = stick->GetRawAxis(0);
    *yStickValue = -stick->GetRawAxis(1);
    *zStickValue = stick->GetRawAxis(2);

    if (stick->GetRawButton(2) and !*driveLatch) {
        *driveToggle = !*driveToggle;
        *driveLatch = true;
    }
    else if (!stick->GetRawButton(2) and *driveLatch) {
        *driveLatch = false;
    }

    if (*driveToggle) {
        *driveGyro = 0;
    }
    else if (!*driveToggle) {
        *driveGyro = ahrs->GetAngle();
    }

    mecanumDrive->DriveCartesian(*yStickValue, *xStickValue, *zStickValue, *driveGyro);

    *gyro = ahrs->GetAngle(); 
    frc::SmartDashboard::PutNumber("gyro", *gyro);

    if (stick->GetRawButton(4)) {
        ahrs->Reset();
    }
}