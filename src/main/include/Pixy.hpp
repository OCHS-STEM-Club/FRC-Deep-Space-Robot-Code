#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <frc/I2C.h>
#include <hal/Hal.h>

#define I2C_ADDRESS 0x54

typedef unsigned char byte;
byte buff[32];

class PixyManager {

private:
frc::I2C *I2CPixy;
frc::Joystick *stick; 


public:
    PixyManager();
    void pixy(); 
};