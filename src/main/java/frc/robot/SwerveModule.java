package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.frc4669;
import frc.robot.Constants.Swerve;
import frc.robot.frc4669.Logging;
import frc.robot.SwerveModuleConfig;

public class SwerveModule { 
    public final SwerveModuleConfig m_config; 
    private DutyCycleEncoder m_steerAbsloute; 
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private TalonFXConfiguration steerMotorConfig; 
    private PositionDutyCycle m_steerPositionCtrl;
    private VelocityDutyCycle m_driveVelocityCtrl;
    private boolean m_isZeroed = false; 

    public SwerveModule(SwerveModuleConfig config) {
        m_config = config;
        m_steerAbsloute = frc4669.GetSRXEncoderOnRIODIO(config.absoulteEncoderCID); 
        m_driveMotor = new TalonFX(config.driveID);
        m_steerMotor = new TalonFX(config.steerID);
        m_steerPositionCtrl = new PositionDutyCycle(0); // Steer PID control
        m_driveVelocityCtrl = new VelocityDutyCycle(0); // Drive PID control

        steerMotorConfig = frc4669.GetFalcon500DefaultConfig();
        steerMotorConfig.MotorOutput.Inverted = config.steerInverted; 
        steerMotorConfig.Feedback.SensorToMechanismRatio = Swerve.kSwerveSteerGearRatio / 360;
        steerMotorConfig.Slot0.kP = config.kpSteer;
        m_steerMotor.getConfigurator().apply(steerMotorConfig);
        m_steerMotor.setPosition(0); // reset cached encoder values between code restarts

        TalonFXConfiguration driveMotorConfig = frc4669.GetFalcon500DefaultConfig();
        driveMotorConfig.MotorOutput.Inverted = config.driveInverted; 
        driveMotorConfig.Feedback.SensorToMechanismRatio = Swerve.kSwerveDriveGearRatio / Swerve.kWheelCircumference;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.Slot0.kP = config.kpDrive;
        // driveMotorConfig.Slot0.kD = config.kdDrive;
        m_driveMotor.getConfigurator().apply(driveMotorConfig);
    }

    public void setState(SwerveModuleState state, boolean usePID, boolean enabled) {
        if (!enabled) return;
        if (!m_isZeroed) return;

        state = frc4669.SwerveOptimizeAngle(state, angle());

        if (usePID) m_driveMotor.setControl(m_driveVelocityCtrl.withVelocity(state.speedMetersPerSecond));
        else m_driveMotor.set(state.speedMetersPerSecond/*/ Swerve.kMaxAttainableSpeed*/);

        m_steerMotor.setControl(m_steerPositionCtrl.withPosition(state.angle.getDegrees()));
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

    public double getSteerAbsPosition() {
        SmartDashboard.putNumber("ENC" + m_steerAbsloute.getSourceChannel(), m_steerAbsloute.getAbsolutePosition()*360);
        return (m_steerAbsloute.getAbsolutePosition() * 360) % 360;
    }

    private double m__outputAngle; 

    public Command setSteerOffset() {
        return Commands.runOnce(() -> {
            // calculate and go to the actual zero position
            double currentAbsAngle = getSteerAbsPosition(); 
            double targetAbsAngle = m_config.steerAlignedAbsPosition * 360; 
            m__outputAngle = (currentAbsAngle-targetAbsAngle);  // I don't know why tf it's current - target and not target - current
            m_steerMotor.setControl(m_steerPositionCtrl.withPosition(m_steerMotor.getPosition().getValueAsDouble() + m__outputAngle));
        }).alongWith(Commands.waitUntil(()-> {
            double currentAngle = angle().getDegrees() % 360.0; 
            return (currentAngle >= m__outputAngle-1 && currentAngle <= m__outputAngle+1) && m_steerMotor.getVelocity().getValueAsDouble() < 0.05; 
        })).andThen( // reset encoder again
            Commands.runOnce(()-> {
                m_steerMotor.setPosition(0); 
                m_steerMotor.setControl(m_steerPositionCtrl.withPosition(0));
                m_isZeroed = true;
            })
        // module is only calibrated once, calibrating again WILL break the module
        ).onlyIf(() -> {return !m_isZeroed;});
    }
}
