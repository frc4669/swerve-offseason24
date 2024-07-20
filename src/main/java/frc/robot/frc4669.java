package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

public class frc4669 {

    /**
     * Applys a joystick lower bound dead zone to an input
     * 
     * @param input input value 
     * @param deadzone lower deadzone
     * @return double input after deadzone is applied
     */
    public static double ApplyJoystickDeadZone(double input, double deadzone) {
        if (Math.abs(input) > deadzone) {
            // linear dead zone 
            return Math.signum(input)*(Math.abs(input)-deadzone)/(1-deadzone);
        }
        return 0; 
    }

    /**
     * Gets default Falcon500 Configuration
     * 
     * @usuage To apply configuration call `motor.getConfigurator().apply(configuration);`
     * 
     * @return TalonFXConfiguration 
     */
    public static TalonFXConfiguration GetFalcon500DefaultConfig() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
        motorConfig.CurrentLimits.SupplyCurrentLimit = 35;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = motorConfig.CurrentLimits.SupplyCurrentLimit; 
        motorConfig.CurrentLimits.SupplyTimeThreshold = 0.1; 

        motorConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Coast; 
        motorConfig.MotorOutput.Inverted = com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive; 
        return motorConfig; 
    }

    public static double squareInput(double input) {
        return Math.pow(input, 2) * Math.signum(input);
    }

    /**
     * Check if we are on the Red Alliance
     * <p>
     * This method can be used to determine when to apply transformations to paths, cmds, etc.
     * 
     * @return `True` if we are on the red alliance, `False` on blue
     */
    public static Boolean IsOnRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
