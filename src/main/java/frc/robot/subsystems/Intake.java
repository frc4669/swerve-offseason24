package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.frc4669;

public class Intake extends SubsystemBase{
    public VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    private TalonFX m_eatRight;
    private TalonFX m_eatLeft;

    public Intake() {
        // config code 
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.1; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        m_eatRight.getConfigurator().apply(slot0Configs);
        m_eatLeft.getConfigurator().apply(slot0Configs);
    }

    public Command SetVelocity(double targetvelocity)
    {
        return runOnce(() -> {
            m_eatRight.setControl(m_request.withVelocity(targetvelocity));
            m_eatLeft.setControl(m_request.withVelocity(targetvelocity));
        });
    }

    public Command pacMan()
    {
        return runOnce(() -> {
            m_eatRight.set(0.5);
            m_eatLeft.set(-0.5);
        });
    };

    public Command imFull()
    {
        return runOnce(() -> {
            m_eatRight.set(0.0);
            m_eatLeft.set(0.0);
        });
    };

    public Command stopMotors()
    {
        return runOnce(() -> {
            m_eatRight.set(0.0);
            m_eatLeft.set(0.0);
        });
    };
}
