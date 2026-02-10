package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {		
    public static final double GEAR_RATIO = 1;

    public static final Transform3d OFFSET = new Transform3d();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOT_SPEED = new InterpolatingDoubleTreeMap();

    /** Motor ids */
    public static enum Motors {
        TOP(12),
        BOTTOM(13);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }
    
    /** Config for shooter motors */
    public static TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(0.5)
                .withKI(0.001)
                .withKD(0.0)
                .withKA(0.1)
                .withKS(0.1)
                .withKV(0.1)) // placeholder values
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimit(60))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0));
    }
}
