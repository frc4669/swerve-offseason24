package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.frc4669;

public class Intake extends SubsystemBase{
    private TalonFX m_eatRight;
    private TalonFX m_eatLeft;



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
