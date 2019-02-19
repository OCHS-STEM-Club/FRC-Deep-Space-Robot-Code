
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "Drive.hpp"
#include "Vision.hpp"
#include "Lift.hpp"
#include "Manipulator.hpp"
#include <ctre/Phoenix.h> 
#include <frc/Joystick.h>
#include <frc/I2C.h>

typedef unsigned char byte;

int step = 0;
int autoNum = 0;

Robot::Robot() {
  driveManager = new DriveManager();
  pixyManager = new PixyManager();
  liftManager = new LiftManager();
  manipulatorManager = new ManipulatorManager();
}

frc::Joystick *stick;

void Robot::RobotInit() {
  m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  stick = new frc::Joystick{ 0 };

/*cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture("1792", 0);
camera.SetResolution(160, 120);
camera.SetFPS(10); */
}



/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

void Robot::RunEric() {
  pixyManager->pixy();

  if (stick->GetRawButton(12)) {
    pixyManager->pixyFunct();
  }
  else {
    driveManager->driveTrain();
  } 

  liftManager->Lift();

  manipulatorManager->manipulate();
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

driveManager->reset();
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

 /*if (autoNum = 0) {
    switch(step) {
      case 0: driveManager->turn(90);
        break;
    } 
 } */
 //driveManager->turn(180);

  /*pixyManager->pixy();

  if (stick->GetRawButton(12)) {
    pixyManager->pixyFunct();
  }
  else {
    driveManager->driveTrain();
  } 

  liftManager->Lift();

  manipulatorManager->manipulate(); */

  RunEric();
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  //driveManager->driveTrain();
  /*pixyManager->pixy();

  if (stick->GetRawButton(12)) {
    pixyManager->pixyFunct();
  }
  else {
    driveManager->driveTrain();
  } 

  liftManager->Lift();

  manipulatorManager->manipulate(); */

  RunEric();
}

void Robot::TestPeriodic() {
  
}

#ifndef RUNNING_FRC_TESTS
START_ROBOT_CLASS(Robot)
#endif