/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include "Drive.hpp"
#include "Vision.hpp"
#include "Lift.hpp"
#include "Manipulator.hpp"
#include <frc/IterativeRobot.h>
#include <frc/SmartDashboard/SendableChooser.h>

  extern int step;


class Robot : public frc::IterativeRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void RunEric();
  Robot();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  DriveManager *driveManager;
  PixyManager *pixyManager;
  LiftManager *liftManager;
  ManipulatorManager *manipulatorManager;
};