package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.frc4669;
import frc.robot.Constants.Swerve;

public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private PositionDutyCycle m_positionDutyCycle;
    private SwerveModuleState m_state = new SwerveModuleState();

    public SwerveModule(int driveID, int steerID, InvertedValue driveInverted, InvertedValue steerInverted, double kpSteer) {
        m_driveMotor = new TalonFX(driveID);
        m_steerMotor = new TalonFX(steerID);
        m_positionDutyCycle = new PositionDutyCycle(0);

        TalonFXConfiguration steerMotorConfig = frc4669.GetFalcon500DefaultConfig();
        steerMotorConfig.MotorOutput.Inverted = steerInverted; 
        steerMotorConfig.Feedback.SensorToMechanismRatio = Swerve.kSwerveSteerGearRatio / 360;
        steerMotorConfig.Slot0.kP = kpSteer;
        m_steerMotor.getConfigurator().apply(steerMotorConfig);

        TalonFXConfiguration driveMotorConfig = frc4669.GetFalcon500DefaultConfig();
        steerMotorConfig.MotorOutput.Inverted = driveInverted; 
        driveMotorConfig.Feedback.SensorToMechanismRatio = Swerve.kSwerveDriveGearRatio / Swerve.kWheelCircumference;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_driveMotor.getConfigurator().apply(driveMotorConfig);
    }

    public void setState(SwerveModuleState state, boolean usePID, boolean enabled) {
        if (!enabled) return; 
        // Add optimization
        this.m_state = SwerveModuleState.optimize(state, angle());

        String id = String.valueOf(m_steerMotor.getDeviceID());

        SmartDashboard.putNumber(id + " target", state.angle.getDegrees());
        // SmartDashboard.putNumber(id + " actual", angle().getDegrees());

        m_driveMotor.set(state.speedMetersPerSecond);
        m_steerMotor.setControl(m_positionDutyCycle.withPosition(state.angle.getDegrees()));
    }

    public SwerveModuleState getState() {
        return m_state; 
    }

    public Rotation2d angle() {
        var pos =  m_steerMotor.getPosition().refresh().getValueAsDouble();
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
