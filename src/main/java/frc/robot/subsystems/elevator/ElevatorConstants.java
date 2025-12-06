package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorConstants {
    public static final double EPSILON = 0.03; // Meters

    public static final double SPROCKET_RADIUS = 0.0412; // Effective pitch radius
    public static final double GEAR_RATIO = 9;
    public static final double SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS * Constants.TAU;
    public static final double METERS_PER_ROTATION = 0.028776; // Approximated using measurement
    public static final double END_EFFECTOR_HEIGHT = 0.54; // Meters
    public static final double CARRIAGE_WEIGHT = 7.55; // kg

    public static final double MAX_ACCEL = 1.5;
    public static final double MAX_SPEED = 1.0; // m/s

    /** Motor IDs */
    public static enum Motors {
        LEFT(20),
        RIGHT(21);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }

    /** Heights for the end effector to score */
    public static enum Heights {
        BASE(END_EFFECTOR_HEIGHT + 0.003), // small offset to prevent stalling (allegedly)
        L1(END_EFFECTOR_HEIGHT + 0.003),
        L2(Units.inchesToMeters(2 * 12 + 7 + 7 / 8.0)),
        L3(Units.inchesToMeters(3 * 12 + 11 + 5 / 8.0)),
        L4(Units.inchesToMeters(6 * 12) - 0.05);
        public final double height;
        private Heights(double height) {
            this.height = height;
        }
    }

    /** Elevator config factory */
    public static TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(0.125)
                .withKV(0.0)
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.05)
                .withKG(0.375))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(metersToRotations(MAX_ACCEL)) // 1.0 m/s^2
                .withMotionMagicCruiseVelocity(metersToRotations(MAX_SPEED))
                .withMotionMagicJerk(320))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimit(60))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0));
    }

    /** Conversion utility */
    public static double rotationsToMeters(double rotations) {
        return rotations * METERS_PER_ROTATION;
    }
    
    /** Conversion utility */
    public static double metersToRotations(double meters) {
        return meters / METERS_PER_ROTATION;
    }
}
