package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

public class IndexerConstants {

    // TODO: Set IDs
    // TODO (Ethan): Set LaserCan_DefaultMeasurement
    public static final int L_MOTOR_ID = 51;
    public static final int L_LASER_ID = 50;
    public static final int R_MOTOR_ID = 49;
    public static final int R_LASER_ID = 48;
    // public static final int LASER_ID_2 = 0;
    public static final double ROLLING_SPEED = 10; // rps
    public static final double MAXIMUM_LASER_DIST = 100;


    public static TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(0.3)
                .withKI(0.001)
                .withKD(0.0)
                .withKA(0.1)
                .withKS(0.1)
                .withKV(0.1)) // placeholder values
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(40))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0));
    }
}