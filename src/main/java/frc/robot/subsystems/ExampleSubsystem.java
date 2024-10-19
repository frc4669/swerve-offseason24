// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.frc4669;

public class ExampleSubsystem extends SubsystemBase {
  // Declear various motors, sensors, PID controllers and other subsystem states here
  // private <class> m_<varibale name>;
  private TalonFX m_motor;     //<-- To declear a motor(Faclon 500)  

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    // Initialize Motors & Set MotorConfigs Here: 
    m_motor = new TalonFX(0); 
    TalonFXConfiguration motorConfig = frc4669.GetFalcon500DefaultConfig();
    // modify the deafult config here, Ex:
    // motorConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive; 
    // motorConfig.Feedback.SensorToMechanismRatio = 1.0/1.0; 
    // motorConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
    m_motor.getConfigurator().apply(motorConfig); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //////////////////////
  // Control Commands //
  //////////////////////

  // Start commands: starts an action on the subsystem, always named with Start<ActionName>
  // always use runOnce except in special cases, so the subsystem isn't blocked by this command forever 
  public Command StartExampleSubsystem(double speed) {
    return runOnce(() -> {
      m_motor.set(speed);  
    });
  }

  // ! It's always either a Start<ActionName> OR a Set<MechanismName>Pos for a motor/mechanism
  // ! never have both unless in special cases, 99% one or the other.

  // SetPos commands, always named with Set<rMechanismName>Pos
  // puts motor/mechanism into a hold at targetPosition (go to targetPosition and stop there)
  // always use runOnce except in special cases, so the subsystem isn't blocked by this command forever 
  // public Command SetExampleSubsystemPos(double targetPosition) {
  //   return runOnce(() -> {
  //     m_motor.setControl(m_motionMagic.WithPosition(targetPosition));
  //   });
  // }

  // Stop commands: stops an action on the subsystem, always named with Stop<ActionName>
  // should always stop what Start<ActionName> started or stop the process of going to Set<Motor/MechnismName>Pos positions
  // normally this means a motor being set to idle (a speed of 0.0) or entering the motor into a PID hold at current position
  public Command StopExampleSubsystem() {
    return runOnce(() -> {
      m_motor.set(0.0);
    });
  }

  // A StopMotors method must be present to set all motors to idle (a speed of 0.0);
  public Command StopMotors() {
    return runOnce(() -> {
      m_motor.set(0.0);
      // m_motor2.set(0.0);
      // m_motor3.set(0.0);
      // ...
    });
  }

  ////////////////////
  // SENSOR METHODS //
  ////////////////////
  // Wait untill <condition> is true
  // public Command WaitUntill<ConditionName>() {
  //   return runOnce(() -> {
  //     // initialization code
  //   }).andThen(run(() -> {
  //     // do somechecking of sensors and store their values to a sensor state decleared in this class
  //   }).until(() -> {
  //     // an if statement to check if the condition is met
  //     // returning true will end the wait
  //     return true;
  //   }));
  // }

}
