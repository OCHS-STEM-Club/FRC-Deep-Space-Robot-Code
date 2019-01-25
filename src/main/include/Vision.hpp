#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <frc/I2C.h>

#define I2C_ADDRESS 0x64

typedef unsigned char byte;

class PixyManager {
private:
frc::I2C *I2CPixy;
frc::Joystick *stick; 

int translate[15];

public:
    PixyManager();
    void pixy(); 
};