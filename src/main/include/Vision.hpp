#pragma once

#include <iostream>
#include <frc/WPILib.h>
#include <frc/Joystick.h>
#include <frc/I2C.h>
#include <cmath>

#include "Drive.hpp"

#define I2C_ADDRESS 0x64

#define PIXY_CENTER_X 160
#define PIXY_DEADBAND_X 5

#define PIXY_DEADBAND_TURN 2.5 //degrees 
#define ANTI_MISSILE_CODE 0.35
#define PIXY_DEADBAND_DISTANCE 0.45

#define PIXY_DISTANCE_X 98 //2ft
#define PIXY_DISTANCE_DEADBAND 4

#define PIXY_DPI 10
#define SIN_HALF_PIXYFOV 0.608761429 //sin(37.5*)
#define DISTANCE_MID_TO_TARGET 5.75 //in

typedef unsigned char byte;

class PixyManager {
private:
DriveManager *driveManager;

frc::I2C *I2CPixy;
frc::Joystick *stick; 

AHRS *ahrs; 

double angle;
int revolutions; 

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

//Finding Angle
double pixyDistanceMidToTarget;
double totalPixelsLeftOfTarget;
double totalPixelsRightOfTarget;
double FOV_Right_in;
double FOV_Left_in;
double angleOppositeDistance_Left;
double angleOppositeDistance_Right;
double angleFromLeftSide;
double angleFromRightSide;
double distanceToTarget_in;

public:
    PixyManager();
    void pixy(); 
    void pixyFunct();
};