package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {
    public static final double EPSILON = 0.01; // Meters
    public static final double SPROCKET_RADIUS = Units.inchesToMeters(0.8785);
    public static final double SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS * Constants.TAU;
    public static final double GEAR_RATIO = 9;
    public static final double END_EFFECTOR_HEIGHT = 0.62; // Meters
    public static final double MAX_SPEED = 0.5;

    public static enum Motors {
        LEFT(20),
        RIGHT(21);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }

    public static enum Heights {
        BASE(END_EFFECTOR_HEIGHT),
        L1(Units.inchesToMeters(1 * 12 + 6) + END_EFFECTOR_HEIGHT),
        L2(Units.inchesToMeters(2 * 12 + 7 + 7 / 8.0) + END_EFFECTOR_HEIGHT),
        L3(Units.inchesToMeters(3 * 12 + 11 + 5 / 8.0) + END_EFFECTOR_HEIGHT),
        L4(Units.inchesToMeters(6 * 12) + END_EFFECTOR_HEIGHT);
        public final double height;
        private Heights(double height) {
            this.height = height;
        }
    }

    public static TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(0.125)
                .withKV(0.0)
                .withKP(5.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKG(0.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(16.0)
                .withMotionMagicCruiseVelocity(heightToRotations(0.2))
                .withMotionMagicJerk(300.0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withSupplyCurrentLimit(30))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0));
    }

    public static double rotationsToHeight(double rotations) {
        return rotations / GEAR_RATIO * SPROCKET_CIRCUMFERENCE;
    }
    
    public static double heightToRotations(double meters) {
        return meters / SPROCKET_CIRCUMFERENCE * GEAR_RATIO;
    }
}
