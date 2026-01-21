package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

public final class ShooterConstants {		
    public static final double SHOOT_SPEED_INCREMENT = 5; // rotation per second

    /** Motor ids */
    public static enum Motors {
        TOP(12),
        BOTTOM(13);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }

    /** Laser ids */
    public static enum Lasers {
        //BACK(30),
        FRONT(31);
        public final int id;
        private Lasers(int id) {
            this.id = id;
        }
    }

    /** Config for shooter motors */
    public static TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(0.3)
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
