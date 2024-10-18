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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.CorrectAxisSwerveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.WPIUtilJNI;
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
  private CorrectAxisSwerveOdometry m_odometry;

  private WPI_TalonSRX m_swerveZeroingEncoder;
  private DutyCycleEncoder m_swervePWMEncoder; 

  private final Field2d m_field = new Field2d(); 

  private double m_prevAngle = 0; 
  private long m_prevTime = 0;

  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain(Vision vision) {
    m_vision = vision;
    // m_swervePWMEncoder = new DutyCycleEncoder(9); 
    // m_swervePWMEncoder.setDutyCycleRange(1/4096, 4096/4096);

    m_gyro = new AHRS(SPI.Port.kMXP);
    m_kinematics = new SwerveDriveKinematics(
      Constants.Swerve.kFrontLeftOffset,
      Constants.Swerve.kFrontRightOffset,
      Constants.Swerve.kRearRightOffset,
      Constants.Swerve.kRearLeftOffset
    );

    m_modules = new SwerveModule[4];
    m_modules[0] = new SwerveModule(Constants.Swerve.M1);
    m_modules[1] = new SwerveModule(Constants.Swerve.M3);
    m_modules[2] = new SwerveModule(Constants.Swerve.M2);
    m_modules[3] = new SwerveModule(Constants.Swerve.M4);

    m_modules[0].resetAzimuth();
    m_modules[1].resetAzimuth();
    m_modules[2].resetAzimuth();
    m_modules[3].resetAzimuth();
    
    // grab the current vision position or assume we start at 0, 0 with yaw 0
    Vision.VisionRobotPos robotStartPose = vision.GetVisionRobotPos().orElse(m_vision.new VisionRobotPos());     
    m_odometry = new CorrectAxisSwerveOdometry(m_kinematics, angleRot2d(), swerveModulePositions(), robotStartPose);

    // Auto set up
    AutoBuilder.configureHolonomic(
      this::getRobotPose, 
      (pos) -> m_odometry.resetPosition(angleRot2d(), swerveModulePositions(), pos), 
      () -> m_kinematics.toChassisSpeeds(m_modules[0].getState(), m_modules[1].getState(), m_modules[2].getState(), m_modules[3].getState()), 
      (speeds) -> {
        speeds.vxMetersPerSecond *= -1.0;
        SmartDashboard.putNumber("Commanded Vel", speeds.vxMetersPerSecond);        
        this.drive(speeds, true);
      }, 
      Constants.Auto.kHolonomicConfig, 
      frc4669::IsOnRedAlliance,
      this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putData(m_field);
  }

  public Command ZeroSwerveModules() {
    return new SequentialCommandGroup(
      m_modules[0].setSteerOffset(this),        
      m_modules[1].setSteerOffset(this),    
      m_modules[2].setSteerOffset(this),    
      m_modules[3].setSteerOffset(this)
    );
    // }).andThen(run(()->{}).until(()-> {
    //   int numFinished = 0;  
    //   for (SwerveModule module : m_modules) {
    //     double currentDeg = module.angle().getDegrees(); 
    //     if (currentDeg >= -10 && currentDeg <= 10) 
    //       numFinished += 1;
    //   }
    //   if (numFinished >= 4) return true;
    //   else return false;
    // }));
    // double currentAbsPos = (((double)m_swerveZeroingEncoder.getSensorCollection().getPulseWidthPosition())/Constants.Swerve.kSRXPlusePerRoatation) * 360;
    // double currentAbsPos = (double)m_swervePWMEncoder.getAbsolutePosition() * 360.0; 
    // System.out.println(m_swervePWMEncoder.getAbsolutePosition());
    // System.out.println(currentAbsPos);
    // m_modules[1].zeroSteering(currentAbsPos % 360,Constants.Swerve.kFrontLeftZero);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro", angle());
    SmartDashboard.putNumber("M1 Azimuth", m_modules[0].angle().getDegrees());
    SmartDashboard.putNumber("M2 Azimuth", m_modules[2].angle().getDegrees());
    SmartDashboard.putNumber("M3 Azimuth", m_modules[1].angle().getDegrees());
    SmartDashboard.putNumber("M4 Azimuth", m_modules[3].angle().getDegrees());

    m_odometry.update(Rotation2d.fromDegrees(-angle()), swerveModulePositions()); 
    // if vision angles exist, fuse it with gyro measurements
    m_vision.GetVisionRobotPos().ifPresent((pos) -> m_odometry.addVisionMeasurement(pos, pos.ts)); 

    m_field.setRobotPose(getRobotPose());

m_modules[0].getSteerAbsPosition();
m_modules[1].getSteerAbsPosition();
m_modules[2].getSteerAbsPosition();
m_modules[3].getSteerAbsPosition();
  }

  // drive using speed inputs
  public void drive(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, angleRot2d());
    SmartDashboard.putNumber("rotation input", rotation);

    // FOR ROBOT RELATIVE:
    // ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, rotation);
    drive(speeds, false);
  }

  // drive using ChassisSpeeds
  public void drive(ChassisSpeeds speeds, boolean usePID) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.kMaxAttainableSpeed);

    m_modules[0].setState(states[0], usePID, true);
    m_modules[1].setState(states[1], usePID, true);
    m_modules[2].setState(states[2], usePID, true);
    m_modules[3].setState(states[3], usePID, true);
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
