package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.frc4669;
import frc.robot.Constants.Swerve;
import frc.robot.SwerveModuleConfig;

public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private PositionDutyCycle m_positionDutyCycle;
    private VelocityDutyCycle m_velocityDutyCycle;
    private SwerveModuleState m_state = new SwerveModuleState();

    public SwerveModule(SwerveModuleConfig config) {
        m_driveMotor = new TalonFX(config.driveID);
        m_steerMotor = new TalonFX(config.steerID);
        m_positionDutyCycle = new PositionDutyCycle(0); // Steer PID control
        m_velocityDutyCycle = new VelocityDutyCycle(0); // Drive PID control

        TalonFXConfiguration steerMotorConfig = frc4669.GetFalcon500DefaultConfig();
        steerMotorConfig.MotorOutput.Inverted = config.steerInverted; 
        steerMotorConfig.Feedback.SensorToMechanismRatio = Swerve.kSwerveSteerGearRatio / 360;
        steerMotorConfig.Slot0.kP = config.kpSteer;
        m_steerMotor.getConfigurator().apply(steerMotorConfig);

        TalonFXConfiguration driveMotorConfig = frc4669.GetFalcon500DefaultConfig();
        steerMotorConfig.MotorOutput.Inverted = config.driveInverted; 
        driveMotorConfig.Feedback.SensorToMechanismRatio = Swerve.kSwerveDriveGearRatio / Swerve.kWheelCircumference;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.Slot0.kP = config.kpDrive;
        driveMotorConfig.Slot0.kD = config.kdDrive;
        m_driveMotor.getConfigurator().apply(driveMotorConfig);
    }

    public void setState(SwerveModuleState state, boolean usePID, boolean enabled) {
        if (!enabled) return;
        
        state = frc4669.SwerveOptimizeAngle(state, angle());

        if (usePID) m_driveMotor.setControl(m_velocityDutyCycle.withVelocity(state.speedMetersPerSecond));
        else m_driveMotor.set(state.speedMetersPerSecond/*/ Swerve.kMaxAttainableSpeed*/);

        m_steerMotor.setControl(m_positionDutyCycle.withPosition(state.angle.getDegrees()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getVelocity().getValueAsDouble(), angle()); 
    }

    public Rotation2d angle() {
        double pos = m_steerMotor.getPosition().refresh().getValueAsDouble();
        return Rotation2d.fromDegrees(pos);
    }

    public double distanceTraveled() {
        return m_driveMotor.getPosition().refresh().getValueAsDouble(); 
    }

    public void zeroSteering(double currentAbsAngle, double targetAbsAngle) {
        double outputAngle = (currentAbsAngle-targetAbsAngle);  // I don't know why tf it's current - target and not target - current
        // it works, so do not change
        m_steerMotor.setControl(m_positionDutyCycle.withPosition(m_steerMotor.getPosition().getValueAsDouble() + outputAngle)); 
    }

    public void resetAzimuth() { // PROBABLY NOT SAFE, REMOVE WHEN ABS ENCODERS ARE ADDED
        m_steerMotor.setPosition(0);
    }
}
