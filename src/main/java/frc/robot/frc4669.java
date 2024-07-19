// DEV NOTE: Everything defined in this file must have usuage documentation
// it is also recommonded to do CTRL+SHIFT+P and run `Fold All` so it's easier to read this file
package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

//** A general library for commonly used, non-season specific functions created by Galileo Robotics */
public class frc4669 {
    /**Applys a joystick lower bound dead zone to an input
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

    /**Gets default Falcon500 Configuration
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

    /**Squares input while retaining the sign (if input is negative, output is negative)
     * 
     * @param input input number
     * @return input^2 * (sign of input)
     */
    public static double squareInput(double input) {
        return Math.pow(input, 2) * Math.signum(input);
    }

    /**Check if we are on the Red Alliance
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

    //** Small custom datalogging library that uses wpilib's DataLogManager */
    public static class Logging {
    //**Starts the DataLogManager with dafaults */
    public static void StartLogger() {
        DataLogManager.start();
    } 
    //** log a message */
    public static <T> void log(T input) {
        log(String.valueOf(input));
    }
    //** log mutiple messages on the same line, seperated by `; `*/
    public static void log(String... messages) {
        log(String.join("; ", messages));
    }
    //** log a message to DataLog */
    public static void log(String message) {
        DataLogManager.log(message);
    }
    //** log a info message with `INFO:` prefix */
    public static <T> void info(T input) {
        log("INFO:" + input);
    }
    //** log a warning with `WARNING:` prefix */
    public static <T> void warn(T input) {
        log("WARNING:" + input);
    }
    //** log a error with `ERROR:` prefix */
    public static <T> void error(T input) {
        log("ERROR:" + input);
    }
    }

}
