// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {
  private static final int kXboxChannel = 0;

  private DriveTrain m_driveTrain;
  private XboxController m_controller;

  @Override
  public void robotInit() {
    // Instantiate the DriveTrain subsystem
    m_driveTrain = new DriveTrain();
    // Instantiate the joystick
    m_controller = new XboxController(kXboxChannel);
  }

  @Override
  public void teleopPeriodic() {
    // Pass joystick inputs to the DriveTrain's mecanumDrive method
    m_driveTrain.mecanumDrive(-m_controller.getLeftX(), -m_controller.getLeftY(), -m_controller.getRightX());
  }
}
