package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final CANSparkMax m_frontLeftMotor;
  private final CANSparkMax m_rearLeftMotor;
  private final CANSparkMax m_frontRightMotor;
  private final CANSparkMax m_rearRightMotor;

  // private TalonSRXSimCollection leftDriveSim;
  // private TalonSRXSimCollection rightDriveSim;

  // private final Field2d m_field = new Field2d();
  // private final MecanumDriveOdometry m_odometry;
  // private final MecanumDriveKinematics driveSim;  

  // NEO motor encoders
  private final RelativeEncoder m_frontLeftEncoder;
  private final RelativeEncoder m_rearLeftEncoder;
  private final RelativeEncoder m_frontRightEncoder;
  private final RelativeEncoder m_rearRightEncoder;

  private final MecanumDriveKinematics m_kinematics;

  private static final int kFrontLeftChannel = 1;
  private static final int kRearLeftChannel = 2;
  private static final int kFrontRightChannel = 3;
  private static final int kRearRightChannel = 4;

  public DriveTrain() {

    // Initialize motor controllers
    m_frontLeftMotor = new CANSparkMax(kFrontLeftChannel, MotorType.kBrushless);
    m_rearLeftMotor = new CANSparkMax(kRearLeftChannel, MotorType.kBrushless);
    m_frontRightMotor = new CANSparkMax(kFrontRightChannel, MotorType.kBrushless);
    m_rearRightMotor = new CANSparkMax(kRearRightChannel, MotorType.kBrushless);

    // Initialize encoders from the NEO motors
    m_frontLeftEncoder = m_frontLeftMotor.getEncoder();
    m_rearLeftEncoder = m_rearLeftMotor.getEncoder();
    m_frontRightEncoder = m_frontRightMotor.getEncoder();
    m_rearRightEncoder = m_rearRightMotor.getEncoder();

    // Configure encoders
    double wheelDiameter = 0.1347; //need to ask
    double ticksPerRevolution = 42;

    // Set distance per tick to convert encoder counts to distance (meters)
    double distancePerTick = Math.PI * wheelDiameter / ticksPerRevolution;
    m_frontLeftEncoder.setPositionConversionFactor(distancePerTick);
    m_rearLeftEncoder.setPositionConversionFactor(distancePerTick);
    m_frontRightEncoder.setPositionConversionFactor(distancePerTick);
    m_rearRightEncoder.setPositionConversionFactor(distancePerTick);

    // Invert the right-side motors to match the direction of travel
    m_frontRightMotor.setInverted(true);
    m_rearRightMotor.setInverted(true);

    // Define wheel locations relative to the robot's center
    Translation2d m_frontLeftLocation = new Translation2d(0.277, 0.277);
    Translation2d m_frontRightLocation = new Translation2d(0.277, -0.277);
    Translation2d m_backLeftLocation = new Translation2d(-0.277, 0.277);
    Translation2d m_backRightLocation = new Translation2d(-0.277, -0.277);

    // Create kinematics object using wheel locations
    m_kinematics = new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
  }

  public void mecanumDrive(double vx, double vy, double rv) {
    // Create an instance of ChassisSpeeds with the given inputs
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rv);

    // Convert chassis speeds to wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

    // Set motor outputs based on wheel speeds
    m_frontLeftMotor.set(wheelSpeeds.frontLeftMetersPerSecond);
    m_frontRightMotor.set(wheelSpeeds.frontRightMetersPerSecond);
    m_rearLeftMotor.set(wheelSpeeds.rearLeftMetersPerSecond);
    m_rearRightMotor.set(wheelSpeeds.rearRightMetersPerSecond);
  }

  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
