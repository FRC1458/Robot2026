package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

public class RollerConstants {
    public static final int MOTOR_ID = 54;
    public static final double ROLL_SPEED = 20;

    public static TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withSupplyCurrentLimit(30))
            .withVoltage(new VoltageConfigs()
            .withPeakForwardVoltage(12.0)
            .withPeakReverseVoltage(-12.0));        
    }
}
