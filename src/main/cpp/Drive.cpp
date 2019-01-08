#include "Drive.hpp"

DriveManager::DriveManager () {
    stick = new Joystick{ 0 };

    driveMotorLeft = new WPI_TalonSRX(1);
    driveMotorRight = new WPI_TalonSRX(2);

    tankDrive = new DifferentialDrive(*driveMotorLeft, *driveMotorRight);

    xStickValue = new double; 
    yStickValue = new double; 
}

void DriveManager::driveTrain() {
    *xStickValue = -stick->GetRawAxis(1);
    *yStickValue = stick->GetRawAxis(2);

    tankDrive->ArcadeDrive(*xStickValue, *yStickValue);
}