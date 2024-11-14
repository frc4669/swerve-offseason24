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

public class Pivit extends SubsystemBase {
    private TalonFX biceps;
    private PositionVoltage bicepy = new PositionVoltage(0);
  public Pivit() {
    biceps = new TalonFX(Constants.Pivit.kMotorCANID); 
    TalonFXConfiguration motorConfig = frc4669.GetFalcon500DefaultConfig();
    motorConfig.Slot0.kP = Constants.Pivit.kP;
    motorConfig.Slot0.kD = Constants.Pivit.kD;     
    motorConfig.Slot0.kI = Constants.Pivit.kI;
    biceps.getConfigurator().apply(motorConfig); 



  }

  @Override
  public void periodic() {
  }

  public Command SetPosition(double pos) {
  
    return runOnce(() -> {
      // code 
      biceps.setControl(bicepy.withPosition(pos));

    });
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