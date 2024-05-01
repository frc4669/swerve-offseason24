// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
  }

  public static class CAN {
    public static final int kSwerveM1Steer = 11;
    public static final int kSwerveM1Drive = 12;
    public static final int kSwerveM2Steer = 13;
    public static final int kSwerveM2Drive = 14;
    public static final int kSwerveM3Steer = 15;
    public static final int kSwerveM3Drive = 16;
    public static final int kSwerveM4Steer = 17;
    public static final int kSwerveM4Drive = 18;
  }

  public static class Swerve {
    public static final double kSRXPlusePerRoatation = 4096.0; 

    public static final double kSwerveVelocityMultiplier = 1.0;

    // 1 motor rot => x output rot 
    public static final double kSwerveSteerGearRatio = 150.0/7.0; 
    public static final double kSwerveDriveGearRatio = 6.75;
    public static final double kWheelCircumference = 0.1016 * Math.PI; // meters

    public static final Translation2d kFrontLeftOffset = new Translation2d(-0.5, 0.5);
    public static final Translation2d kFrontRightOffset = new Translation2d(0.5, 0.5);
    public static final Translation2d kRearRightOffset = new Translation2d(0.5, -0.5);
    public static final Translation2d kRearLeftOffset = new Translation2d(-0.5, -0.5);

    public static final double kFrontLeftZero = (1220 / kSRXPlusePerRoatation) * 360;

    public static final double ksDrive = 0;
    public static final double kvDrive = 0;
    public static final double kaDrive = 0;

    public static final double kpDrive = 0;
    public static final double kdDrive = 0;

    public static final double kpSteer = 0.022; // Module 2
    public static final double kdSteer = 0;
  }
}
