package frc.robot.subsystems;
import frc.robot.frc4669;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants; 

public class Elevator extends SubsystemBase {
    private TalonFX hamstring;
    private PositionVoltage hamstringy = new PositionVoltage(0);
  public Elevator() {
    hamstring = new TalonFX(Constants.Elevator.kMotorCANID); 
    TalonFXConfiguration motorConfig = frc4669.GetFalcon500DefaultConfig();
    motorConfig.Slot0.kP = Constants.Elevator.kP; 
    motorConfig.Slot0.kD = Constants.Elevator.kD;    
    motorConfig.Slot0.kI = Constants.Elevator.kI;
    hamstring.getConfigurator().apply(motorConfig); 
  }

  @Override
  public void periodic() {
  }

  public Command SetPosition(double pos) {
    return runOnce(() -> {
      hamstring.setControl(hamstringy.withPosition(pos));
    });
  }

  public Command StopMotor() {
    return runOnce(() -> {
      hamstring.set(0.0);
    });
  }
}