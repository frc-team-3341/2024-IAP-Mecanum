// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private static final int kFrontLeftChannel = 2;
  private static final int kRearLeftChannel = 3;
  private static final int kFrontRightChannel = 1;
  private static final int kRearRightChannel = 0;

  private final PWMSparkMax m_frontLeftMotor;
  private final PWMSparkMax m_rearLeftMotor;
  private final PWMSparkMax m_frontRightMotor;
  private final PWMSparkMax m_rearRightMotor;

  private final MecanumDriveKinematics m_kinematics;

  public DriveTrain() {
    // Initialize motors
    m_frontLeftMotor = new PWMSparkMax(kFrontLeftChannel);
    m_rearLeftMotor = new PWMSparkMax(kRearLeftChannel);
    m_frontRightMotor = new PWMSparkMax(kFrontRightChannel);
    m_rearRightMotor = new PWMSparkMax(kRearRightChannel);

    // Invert the right-side motors
    m_frontRightMotor.setInverted(true);
    m_rearRightMotor.setInverted(true);

    // Define wheel locations relative to the robot's center
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Create kinematics object using wheel locations
    m_kinematics = new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
  }

  public void mecanumDrive(double x, double y, double rotation) {
    // Create an instance of ChassisSpeeds with the given inputs
    ChassisSpeeds speeds = new ChassisSpeeds(x, y, rotation);

    // Convert chassis speeds to wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

    // // Get the maximum wheel speed to normalize if necessary
    // double maxSpeed = Math.max(
    //   Math.max(Math.abs(wheelSpeeds.frontLeftMetersPerSecond), Math.abs(wheelSpeeds.frontRightMetersPerSecond)),
    //   Math.max(Math.abs(wheelSpeeds.rearLeftMetersPerSecond), Math.abs(wheelSpeeds.rearRightMetersPerSecond))
    // );

    // // Normalize speeds if any wheel speed exceeds 1.0
    // if (maxSpeed > 1.0) {
    //   wheelSpeeds.frontLeftMetersPerSecond /= maxSpeed;
    //   wheelSpeeds.frontRightMetersPerSecond /= maxSpeed;
    //   wheelSpeeds.rearLeftMetersPerSecond /= maxSpeed;
    //   wheelSpeeds.rearRightMetersPerSecond /= maxSpeed;
    // }

    // Set motor outputs
    m_frontLeftMotor.set(wheelSpeeds.frontLeftMetersPerSecond);
    m_frontRightMotor.set(wheelSpeeds.frontRightMetersPerSecond);
    m_rearLeftMotor.set(wheelSpeeds.rearLeftMetersPerSecond);
    m_rearRightMotor.set(wheelSpeeds.rearRightMetersPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
