#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <frc/I2C.h>
#include <cmath>

#include "Drive.hpp"

#define I2C_ADDRESS 0x64

#define PIXY_CENTER_X 165
#define PIXY_DEADBAND_X 5

#define PIXY_DEADBAND_TURN 2.5 //degrees 
#define ANTI_MISSILE_CODE 0.35
#define PIXY_DEADBAND_DISTANCE 0.45

#define PIXY_DISTANCE_X 98 //2ft
#define PIXY_DISTANCE_DEADBAND 4

typedef unsigned char byte;

class PixyManager {
private:
DriveManager *driveManager;

frc::I2C *I2CPixy;
frc::Joystick *stick; 

AHRS *ahrs; 

double angle;
int revoultions; 

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
double strafeCorrectionToIdealCenter;
bool goodTargets; 

//bool leftTargetBig;
double bigSize;
double smallSize;
double turnOffset;
double turnWant;
double turnCorrection; 

double pixyDistanceBetweenTargets;
double pixyDistanceCorrection;
double driveCorrection;

public:
    PixyManager();
    void pixy(); 
    void pixyFunct();
};