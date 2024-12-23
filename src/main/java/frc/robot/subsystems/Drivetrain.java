// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.frc4669;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private TalonFX m_driveMotor; 
  
  private TalonFX m_SteerMotor;
  private com.ctre.phoenix6.controls.PositionDutyCycle m_SteerMotorPID; 
  // private WPI_TalonSRX m_steerEncoder;


  public Drivetrain() {
    m_SteerMotor = new TalonFX(Constants.Swerve.M1.steerID); 
    m_SteerMotor.setSafetyEnabled(false);
    TalonFXConfiguration steerMotorConfig = frc4669.GetFalcon500DefaultConfig(); 
    steerMotorConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kSwerveSteerGearRatio; 
    steerMotorConfig.Slot0.kP = 10.0;
    m_SteerMotor.getConfigurator().apply(steerMotorConfig);

    m_SteerMotorPID = new PositionDutyCycle(0); 


    m_driveMotor = new TalonFX(Constants.Swerve.M1.steerID); 
    m_driveMotor.setSafetyEnabled(false);
    TalonFXConfiguration driveMotorConfig = frc4669.GetFalcon500DefaultConfig();
    driveMotorConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kSwerveDriveGearRatio;
    m_driveMotor.getConfigurator().apply(driveMotorConfig);

    // m_steerEncoder = new WPI_TalonSRX(0);
    // m_steerEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetSwerveOutput(double steer, double power) {
    this.m_SteerMotor.setControl(m_SteerMotorPID.withPosition(steer));
    this.m_driveMotor.set(power);
  }
}
