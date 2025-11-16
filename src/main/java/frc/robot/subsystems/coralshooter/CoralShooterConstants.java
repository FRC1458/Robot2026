package frc.robot.subsystems.coralshooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

public class CoralShooterConstants {
    public static final double MAX_SPEED = 1; // rotations per second

    public static enum Motors {
        LEFT(12),
        RIGHT(13);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }

    public static enum Lasers {
        BACK(30),
        FRONT(31);
        public final int id;
        private Lasers(int id) {
            this.id = id;
        }
    }

    public static TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(0.125)
                .withKV(0.0)
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.05)
                .withKG(0.3))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withSupplyCurrentLimit(30))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0));
    }
}
