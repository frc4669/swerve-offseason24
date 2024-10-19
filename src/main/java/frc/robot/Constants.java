// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig; 
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kMovementMultiplier = 0.5;
    public static final double kRotationMultiplier = 0.5;
  }

  public static class Swerve {
    public static final double kSRXPlusePerRoatation = 4096.0; 

    public static final double kSpeedLimit = 1.5;// 3.0; // m/s
    public static final double kMaxAttainableSpeed = 5.0; // m/s
    public static final double kSpeedFactor = kSpeedLimit / kMaxAttainableSpeed;

    public static final double kMaxAngularSpeed = 2.0;// 2.4; // rad/s (<motor vel at max voltage> * 2pi) / <gear ratio>

    // 1 motor rot => x output rot 
    public static final double kSwerveSteerGearRatio = 150.0/7.0; 
    public static final double kSwerveDriveGearRatio = 6.75;
    public static final double kWheelCircumference = 0.1016 * Math.PI; // meters

    public static final double kModuleXOffsetAbs = 0.277; 
    public static final double kModuleYOffsetAbs = 0.296; 
    public static final double kModuleDistanceToRobotCenter = Math.sqrt(Math.pow(kModuleXOffsetAbs, 2) + Math.pow(kModuleYOffsetAbs, 2)); 
    public static final Translation2d kFrontLeftOffset = new Translation2d(-kModuleXOffsetAbs, kModuleYOffsetAbs);
    public static final Translation2d kFrontRightOffset = new Translation2d(-kModuleXOffsetAbs, -kModuleYOffsetAbs); // SWAPPED WITH REARLEFT
    public static final Translation2d kRearRightOffset = new Translation2d(kModuleXOffsetAbs, -kModuleYOffsetAbs);
    public static final Translation2d kRearLeftOffset = new Translation2d(kModuleXOffsetAbs, kModuleYOffsetAbs);
    public static final double kFrontLeftZero = (1220 / kSRXPlusePerRoatation) * 360;

    public static final double ksDrive = 0;
    public static final double kvDrive = 0;
    public static final double kaDrive = 0;

    public static final double kpDrive = 0.8; // 0.75
    public static final double kdDrive = 0;

    public static final SwerveModuleConfig M1 = new SwerveModuleConfig(
      12, InvertedValue.CounterClockwise_Positive, kpDrive, kdDrive,
      11, InvertedValue.CounterClockwise_Positive, 0.015, 
      9, 0.2076
    );

    public static final SwerveModuleConfig M2 = new SwerveModuleConfig(
      14, InvertedValue.CounterClockwise_Positive, kpDrive, kdDrive,
      13, InvertedValue.CounterClockwise_Positive, 0.013, 
      8, 0.4906
    );

    public static final SwerveModuleConfig M3 = new SwerveModuleConfig(
      16, InvertedValue.CounterClockwise_Positive, kpDrive, kdDrive,
      15, InvertedValue.CounterClockwise_Positive, 0.017, 
      7, 0.1715
    );

    public static final SwerveModuleConfig M4 = new SwerveModuleConfig(
      18, InvertedValue.CounterClockwise_Positive, kpDrive, kdDrive,
      17, InvertedValue.CounterClockwise_Positive, 0.013, 
      6, 0.1667
    );
  }

  public static class Auto {
    public static final double kAutoMaxVelocity = 0.3; // m/s
    public static final double kAutoMaxAccel = 0.15; // m/s^2
    public static final double kAutoMaxAngularVelocity = 0.5; // rad/s 
    public static final double kAutoMaxAngularAccel = 0.25; // rad/s^2

    public static final HolonomicPathFollowerConfig kHolonomicConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(1, 0.0, 0.0), // Robot Translation PID constants
      new PIDConstants(1.0, 0.0, 0.0), // Robot Rotation PID constants
      6, // max speed per module m/sec
      Swerve.kModuleDistanceToRobotCenter, // distance from furthest module to center of robot
      new ReplanningConfig() // Default path replanning config
    ); 
    
  }
}