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

public class elevator extends SubsystemBase {
    private TalonFX hamstring;
    private PositionVoltage hamstringy = new PositionVoltage(0);
  public elevator() {
    hamstring = new TalonFX(0); 
    TalonFXConfiguration motorConfig = frc4669.GetFalcon500DefaultConfig();
    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kD = 0;     
    motorConfig.Slot0.kI = 0;
    hamstring.getConfigurator().apply(motorConfig); 
  }

  @Override
  public void periodic() {
  }

  public Command SetPosition(double pos) {
  
    return runOnce(() -> {
      // code 
      hamstring.setControl(hamstringy.withPosition(pos));

    });
  }

  public Command Stopham() {
    return runOnce(() -> {
      hamstring.set(0.0);
    });
  }
}