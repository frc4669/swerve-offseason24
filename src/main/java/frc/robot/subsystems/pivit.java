package frc.robot.subsystems;
import frc.robot.frc4669;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
public class pivit extends SubsystemBase {
    private TalonFX biceps;
  public pivit() {
    biceps = new TalonFX(0); 
    TalonFXConfiguration motorConfig = frc4669.GetFalcon500DefaultConfig();
    biceps.getConfigurator().apply(motorConfig); 
  }
  @Override
  public void periodic() {
  }
  public Command Startme(double speed) {
    return runOnce(() -> {
      biceps.set(0.5);  
    });
  }
  public Command Stopme() {
    return runOnce(() -> {
      biceps.set(0.0);
    });
  }
  public Command Stopbiceps() {
    return runOnce(() -> {
      biceps.set(0.0);
    });
  }
}