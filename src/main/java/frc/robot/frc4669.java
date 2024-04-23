package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class frc4669 {
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
}
