// DEV NOTE: Everything defined in this file must have usuage documentation
// it is also recommonded to do CTRL+K then CTRL-0 so it's easier to read this file
package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    /**find shortest path of rotation, assuming angles are from 0 to 360
     * <p>
     * This function Implements: Isaac (https://math.stackexchange.com/users/72/isaac), Shortest way to achieve target angle, URL (version: 2012-02-17): https://math.stackexchange.com/q/110236
     * 
     * @param target target angle in degrees [0, 360]
     * @param current current angle in degrees [0, 360]
     * @return double transformation to closest angle in degrees, positive clockwise, negative CC
     */
    public static double CalculateShortestPathOfRotation(double target, double current) {
        double alpha = target - current;
        double beta = alpha + 360.0;
        double gamma = alpha - 360.0; 

        double alphaAbs = Math.abs(alpha);
        double betaAbs = Math.abs(beta);
        double gammaAbs = Math.abs(gamma);

        // determine which value is shortest
        if (alphaAbs < betaAbs && alphaAbs < gammaAbs) return alpha;
        
        // code will not reach here unless: betaAbs < alphaAbs and gammaAbs < alphaAbs 
        else if (betaAbs < gammaAbs) return beta; 
        
        // code will not reach here unless: gammaAbs < betaAbs and gammaAbs < alphaAbs 
        else return gamma;
    }

    /**Minimize the change in heading(angle) for the desired swerve module state.
     * This will ensure that a steer motor is never going to rotate more than 90 degrees.
     * The returned SwerveModuleState's angle can be directly used as motor position.
     * 
     * @param desiredState the unoptimized state that is targeted 
     * @param currentSteerMotorAngle the current module's steer motor angle. (-∞, ∞)
     * @return SwerveModuleState the optimized state
     * @apiNote This is a custom implenmentation of the optimization algorithm. To fix WPILIB's implenmentation causing full 180 degree is certain circumstances.
     */
    public static SwerveModuleState SwerveOptimizeAngle(SwerveModuleState desiredState, Rotation2d currentSteerMotorAngle) {
        // convert any negative angles to positive angle equivalents 
        double desiredAngle = desiredState.angle.getDegrees();
        while (desiredAngle < 0) desiredAngle += 360.0; 
        desiredAngle = desiredAngle % 360.0;

        double currentAngle = currentSteerMotorAngle.getDegrees();
        while (currentAngle < 0) currentAngle += 360.0;
        currentAngle = currentAngle % 360.0;

        // calculate the other supplementary angle 
        double altAngle = (desiredAngle + 180.0) % 360.0;

        // find how to get to those 2 angles
        double transformToDesired = CalculateShortestPathOfRotation(desiredAngle, currentAngle);
        double transformToAlt = CalculateShortestPathOfRotation(altAngle, currentAngle);
        
        // find which of the 2 possible angles have a shorter travel distance
        double resTransform = transformToDesired; 
        // no need to invert direction if going with the already desired angle
        if (Math.abs(transformToAlt) < Math.abs(transformToDesired)) {
            resTransform = transformToAlt;
            desiredState.speedMetersPerSecond *= -1; // invert direction if using supplenmentry other angle
        }

        desiredState.angle = Rotation2d.fromDegrees(currentSteerMotorAngle.getDegrees() + resTransform);
        return desiredState; 
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
