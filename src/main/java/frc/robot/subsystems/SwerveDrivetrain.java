// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.Swerve;

import frc.robot.Vision;
import frc.robot.frc4669;

public class SwerveDrivetrain extends SubsystemBase {
  private Vision m_vision; 

  private SwerveModule[] m_modules; // Clockwise from front left

  private AHRS m_gyro;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDrivePoseEstimator m_odometry;

  private WPI_TalonSRX m_swerveZeroingEncoder;
  private DutyCycleEncoder m_swervePWMEncoder; 

  private final Field2d m_field = new Field2d(); 

  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain(Vision vision) {
    m_vision = vision;
    m_swerveZeroingEncoder = new WPI_TalonSRX(21);
    m_swervePWMEncoder = new DutyCycleEncoder(9); 
    m_swervePWMEncoder.setDutyCycleRange(1/4096, 4096/4096);
    m_swerveZeroingEncoder.configFeedbackNotContinuous(true, 0);

    m_gyro = new AHRS(SPI.Port.kMXP);
    m_kinematics = new SwerveDriveKinematics(
      Constants.Swerve.kFrontLeftOffset,
      Constants.Swerve.kFrontRightOffset,
      Constants.Swerve.kRearRightOffset,
      Constants.Swerve.kRearLeftOffset
    );

    m_modules = new SwerveModule[4];
    m_modules[0] = new SwerveModule(Constants.CAN.kSwerveM1Drive, Constants.CAN.kSwerveM1Steer, Constants.Swerve.kM1DriveInverted, Constants.Swerve.kM1SteerInverted, Constants.Swerve.kM1PSteer);
    m_modules[1] = new SwerveModule(Constants.CAN.kSwerveM3Drive, Constants.CAN.kSwerveM3Steer, Constants.Swerve.kM3DriveInverted, Constants.Swerve.kM3SteerInverted, Constants.Swerve.kM3PSteer);
    m_modules[2] = new SwerveModule(Constants.CAN.kSwerveM2Drive, Constants.CAN.kSwerveM2Steer, Constants.Swerve.kM2DriveInverted, Constants.Swerve.kM2SteerInverted, Constants.Swerve.kM2PSteer);
    m_modules[3] = new SwerveModule(Constants.CAN.kSwerveM4Drive, Constants.CAN.kSwerveM4Steer, Constants.Swerve.kM4DriveInverted, Constants.Swerve.kM4SteerInverted, Constants.Swerve.kM4PSteer);
  
    m_modules[0].resetAzimuth();
    m_modules[1].resetAzimuth();
    m_modules[2].resetAzimuth();
    m_modules[3].resetAzimuth();
    
    // grab the current vision position or assume we start at 0, 0 with yaw 0
    Vision.VisionRobotPos robotStartPose = vision.GetVisionRobotPos().orElse(m_vision.new VisionRobotPos());     
    m_odometry = new SwerveDrivePoseEstimator(m_kinematics, angleRot2d(), swerveModulePositions(), robotStartPose);

    // Auto set up
    AutoBuilder.configureHolonomic(
      m_odometry::getEstimatedPosition, 
      (pos) -> m_odometry.resetPosition(angleRot2d(), swerveModulePositions(), pos), 
      () -> m_kinematics.toChassisSpeeds(m_modules[0].getState(), m_modules[1].getState(), m_modules[2].getState(), m_modules[3].getState()), 
      this::drive, 
      Constants.Auto.kHolonomicConfig, 
      frc4669::IsOnRedAlliance,
      this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putData(m_field);
  }

  public void ZeroSwerveModules() {
    // double currentAbsPos = (((double)m_swerveZeroingEncoder.getSensorCollection().getPulseWidthPosition())/Constants.Swerve.kSRXPlusePerRoatation) * 360;
    double currentAbsPos = (double)m_swervePWMEncoder.getAbsolutePosition() * 360.0; 
    System.out.println(m_swervePWMEncoder.getAbsolutePosition());
    System.out.println(currentAbsPos);
    m_modules[1].zeroSteering(currentAbsPos % 360,Constants.Swerve.kFrontLeftZero);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro", angle());
    SmartDashboard.putNumber("M1 Azimuth", m_modules[0].angle().getDegrees());
    SmartDashboard.putNumber("M2 Azimuth", m_modules[2].angle().getDegrees());
    SmartDashboard.putNumber("M3 Azimuth", m_modules[1].angle().getDegrees());
    SmartDashboard.putNumber("M4 Azimuth", m_modules[3].angle().getDegrees());

    m_odometry.update(angleRot2d(), swerveModulePositions()); 
    // if vision angles exist, fuse it with gyro measurements
    m_vision.GetVisionRobotPos().ifPresent((pos) -> m_odometry.addVisionMeasurement(pos, pos.ts)); 

    m_field.setRobotPose(getRobotPose());
  }

  // drive using speed inputs
  public void drive(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, angleRot2d());
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(0));
    drive(speeds);
  }

  // drive using ChassisSpeeds
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.kSwerveVelocityMultiplier);

    m_modules[0].setState(states[0], false, true);
    m_modules[1].setState(states[1], false, true);
    m_modules[2].setState(states[2], false, true);
    m_modules[3].setState(states[3], false, true);

    // for (int i = 0; i < 4; i++) {
      // m_modules[i].setState(states[i], false);
    // }
  }

  public void resetSteeringPositions() {
    m_modules[0].setState(new SwerveModuleState(0, new Rotation2d(0)), false, true);
    m_modules[1].setState(new SwerveModuleState(0, new Rotation2d(0)), false, true);
    m_modules[2].setState(new SwerveModuleState(0, new Rotation2d(0)), false, true);
    m_modules[3].setState(new SwerveModuleState(0, new Rotation2d(0)), false, true);
  }

  // get gyro angle
  public double angle() {
    return m_gyro.getAngle();
  }

  public Rotation2d angleRot2d() {
    return Rotation2d.fromDegrees(angle());
  }

  public void resetAngle() {
    m_gyro.reset();
  }

  // get a list of all module positions (distance traveled, steer angle) 
  public SwerveModulePosition[] swerveModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(m_modules[0].distanceTraveled(), m_modules[0].angle()), 
      new SwerveModulePosition(m_modules[1].distanceTraveled(), m_modules[1].angle()), 
      new SwerveModulePosition(m_modules[2].distanceTraveled(), m_modules[2].angle()), 
      new SwerveModulePosition(m_modules[3].distanceTraveled(), m_modules[3].angle())
    };
  }

  public Pose2d getRobotPose() {
    return m_odometry.getEstimatedPosition(); 
  }
  
}
