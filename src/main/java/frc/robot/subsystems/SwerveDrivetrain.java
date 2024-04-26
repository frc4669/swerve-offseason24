// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.Swerve;

public class SwerveDrivetrain extends SubsystemBase {
  private SwerveModule[] m_modules; // Clockwise from front left

  private AHRS m_gyro;
  private SwerveDriveKinematics m_kinematics;

  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain() {
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_kinematics = new SwerveDriveKinematics(
      Constants.Swerve.kFrontLeftOffset,
      Constants.Swerve.kFrontRightOffset,
      Constants.Swerve.kRearRightOffset,
      Constants.Swerve.kRearLeftOffset
    );

    m_modules = new SwerveModule[4];
    m_modules[0] = new SwerveModule(Constants.CAN.kSwerveM1Drive, Constants.CAN.kSwerveM1Steer);
    m_modules[1] = new SwerveModule(Constants.CAN.kSwerveM2Drive, Constants.CAN.kSwerveM2Steer);
    m_modules[2] = new SwerveModule(Constants.CAN.kSwerveM3Drive, Constants.CAN.kSwerveM3Steer);
    m_modules[3] = new SwerveModule(Constants.CAN.kSwerveM4Drive, Constants.CAN.kSwerveM4Steer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(angle()));
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.kSwerveVelocityMultiplier);

    for (int i = 0; i < 4; i++) {
      m_modules[i].setState(states[i], false);
    }
  }

  public double angle() {
    return m_gyro.getAngle();
  }

  public void resetAngle(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }
}
