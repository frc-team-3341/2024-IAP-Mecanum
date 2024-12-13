// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class MecanumDrive extends Command {

  /** Creates a new MecanumDrive. */
  DriveTrain dt;
  XboxController controller;
  
  public MecanumDrive(DriveTrain dt, XboxController controller2) {
    this.dt = dt;
    this.controller = controller2;

    addRequirements(dt);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.mecanumDrive(0.0, 0.0, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPowerRaw = controller.getRawAxis(0);     //x-axis of left joystick
    double rightPowerRaw = controller.getRawAxis(1);    //y-axis of left joystick
    double rotationPowerRaw = controller.getRawAxis(2); //x-axis of right joystick
    dt.mecanumDrive(leftPowerRaw*0.3, rightPowerRaw*0.3, rotationPowerRaw*0.3); //default constant multiplier
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.mecanumDrive(0.0, 0.0, 0.0);
    dt.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
} 