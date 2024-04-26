package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.frc4669;
import frc.robot.Constants.Swerve;

public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private PositionDutyCycle m_positionDutyCycle;

    public SwerveModule(int driveID, int steerID) {
        m_driveMotor = new TalonFX(driveID);
        m_steerMotor = new TalonFX(steerID);
        m_positionDutyCycle = new PositionDutyCycle(0);

        TalonFXConfiguration steerMotorConfig = frc4669.GetFalcon500DefaultConfig();
        steerMotorConfig.Feedback.SensorToMechanismRatio = 360 / Swerve.kSwerveSteerGearRatio;
        steerMotorConfig.Slot0.kP = Swerve.kpSteer;
        m_steerMotor.getConfigurator().apply(steerMotorConfig);

        TalonFXConfiguration driveMotorConfig = frc4669.GetFalcon500DefaultConfig();
        driveMotorConfig.Feedback.SensorToMechanismRatio = Swerve.kWheelCircumference / Swerve.kSwerveDriveGearRatio;
        m_driveMotor.getConfigurator().apply(driveMotorConfig);
    }

    public void setState(SwerveModuleState state, boolean usePID) {
        // Add optimization
        m_driveMotor.set(state.speedMetersPerSecond / Swerve.kSwerveVelocityMultiplier);
        m_steerMotor.setControl(m_positionDutyCycle.withPosition(state.angle.getDegrees()));
    }
}
