// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.frc4669;
import frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private TalonFX m_driveMotor; 
  private TalonFX m_SteerMotor;

  public Drivetrain() {
    m_SteerMotor = new TalonFX(CAN.kSwerveM4Steer); 
    m_SteerMotor.setSafetyEnabled(false);
    TalonFXConfiguration steerMotorConfig = frc4669.GetFalcon500DefaultConfig(); 
    m_SteerMotor.getConfigurator().apply(steerMotorConfig);

    m_driveMotor = new TalonFX(CAN.kSwerveM4Drive); 
    m_driveMotor.setSafetyEnabled(false);
    TalonFXConfiguration driveMotorConfig = frc4669.GetFalcon500DefaultConfig();
    m_driveMotor.getConfigurator().apply(driveMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetSwerveOutput(double steer, double power) {
    this.m_SteerMotor.set(steer);
    this.m_driveMotor.set(power);
  }
}
