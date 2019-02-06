#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

class ManipulatorManager {
    private:
    frc::Joystick *stick; 
    frc::XboxController *xbox;

    WPI_TalonSRX *armMotor; 
    WPI_TalonSRX *extenderMotor;

    double *armSpeed;

    public:
    ManipulatorManager();
    void manipulate();
};