#include "Drive.hpp"

DriveManager::DriveManager () {
    stick = new frc::Joystick{ 0 };

    driveMotorLeft = new WPI_TalonSRX(1);
    driveMotorRight = new WPI_TalonSRX(2);

  tankDrive = new frc::DifferentialDrive(*driveMotorLeft, *driveMotorRight);

    xStickValue = new double; 
    yStickValue = new double; 
}

void DriveManager::driveTrain() {
    *xStickValue = -stick->GetRawAxis(1);
    *yStickValue = stick->GetRawAxis(2);

    //driveMotorLeft

    tankDrive->ArcadeDrive(*xStickValue, *yStickValue);
}