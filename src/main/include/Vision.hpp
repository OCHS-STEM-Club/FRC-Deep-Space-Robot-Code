#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <frc/I2C.h>

#include "Drive.hpp"

#define I2C_ADDRESS 0x64

#define PIXY_CENTER_X 170
#define PIXY_DEADBAND_X 5
#define ANTI_MISSILE_CODE 0.35

typedef unsigned char byte;

class PixyManager {
private:
DriveManager *driveManager;

frc::I2C *I2CPixy;
frc::Joystick *stick; 

int translate[15];

float Pixyx1; 
float Pixyy1; 
float Pixyw1; 
float Pixyh1; 

float Pixyx2; 
float Pixyy2; 
float Pixyw2; 
float Pixyh2;

double xCenter;
double target;
double distanceToIdealCenter;
double straifCorrectionToIdealCenter;
bool goodTargets; 

bool leftTargetBig;
double bigSize;
double smallSize;
double turnOffset;
double turnWant;

public:
    PixyManager();
    void pixy(); 
    void pixyFunct();
};