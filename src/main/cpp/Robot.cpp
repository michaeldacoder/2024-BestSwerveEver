// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "swerve/Swerve.hpp"
#include <math.h>
#include <iostream>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

float len = 30;
float width = 30;

Swerve DRIVE {len, width};
frc::Joystick Jostick { 0 };

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  /* Standard config for logitech controller. 
     Left Joystick X + Y controls strafe + forwards respectively
     Right Joystick X controls rotation */
  float y = Jostick.GetY();
  float x = Jostick.GetX();
  float x2 = Jostick.GetRawAxis(4);


  DRIVE.drive(-y, x, x2, 0);
}

void Robot::DisabledInit() { DRIVE.clear_swerve_memory(); }

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
