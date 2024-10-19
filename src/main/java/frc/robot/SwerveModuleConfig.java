package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveModuleConfig {
  public int driveID;
  public InvertedValue driveInverted;
  public double kpDrive;
  public double kdDrive;

  public int steerID;
  public InvertedValue steerInverted;
  public double kpSteer;

  public int absoulteEncoderCID;
  public double steerAlignedAbsPosition; // rotations 

  public SwerveModuleConfig(int driveID, 
    InvertedValue driveInverted, 
    double kpDrive, 
    double kdDrive, 
    int steerID, 
    InvertedValue steerInverted, 
    double kpSteer, 
    int absoulteEncoderCID, 
    double steerAlignedAbsPosition) 
    {
    this.driveID = driveID;
    this.driveInverted = driveInverted;
    this.kpDrive = kpDrive;
    this.kdDrive = kdDrive;

    this.steerID = steerID;
    this.steerInverted = steerInverted;
    this.kpSteer = kpSteer;

    this.absoulteEncoderCID = absoulteEncoderCID; 
    this.steerAlignedAbsPosition = steerAlignedAbsPosition;
  }
}