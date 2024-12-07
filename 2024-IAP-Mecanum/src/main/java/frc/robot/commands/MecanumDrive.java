// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class MecanumDrive extends Command {
  /** Creates a new MecanumDrive. */

  DriveTrain dt;
  Joystick joy; 

  public MecanumDrive(DriveTrain dt, Joystick j) {
    this.dt = dt;
    this.joy = j;

    addRequirements(dt);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.mecanumDrive(0.0, 0.0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPowerRaw = joy.getRawAxis(1);
    double rightPowerRaw = joy.getRawAxis(5);

    dt.mecanumDrive(leftPowerRaw*0.3, rightPowerRaw*0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.mecanumDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}