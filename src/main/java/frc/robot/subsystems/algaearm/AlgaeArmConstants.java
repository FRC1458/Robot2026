package frc.robot.subsystems.algaearm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class AlgaeArmConstants {
    public static final double EPSILON = 0.01; // Meters
    
    public static final double SPROCKET_RADIUS = Units.inchesToMeters(0.8785); // Effective pitch radius
    public static final double SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS * Constants.TAU;
    public static final double GEAR_RATIO = 24;

    private static final double RADIANCONVERSION = 180.0/Math.PI;
    private static final double RADIANS_PER_ROTATION = 0.26179938779; // Approximated via counting both gears used + the gear box. gear box is 2:1

    public static final double MAX_ANGLE = 90.0 * RADIANCONVERSION; // Max angle in Radians
    public static final double MIN_ANGLE = 0.0 * RADIANCONVERSION; // Min angle in Radians

    /** Motor ID */
    public static enum Motors {
        PIVOT_MOTOR(26);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }

    /** AlgaeArm config factory */
    public static TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(0.125)
                .withKV(0.0)
                .withKP(5.0)
                .withKI(0.0)
                .withKD(0.1)
                .withKG(0.1)
                .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(72.5)
                .withMotionMagicCruiseVelocity(10.0)
                .withMotionMagicJerk(1600.0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(10.0)
                .withSupplyCurrentLimit(10.0))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0));
    }

    /** Conversion utility */
    public static double angleToRotations(double angle) {
        return angle / RADIANS_PER_ROTATION;
    }

    /** Conversion utility */
    public static double rotationsToAngle(double rotations) {
        return rotations * RADIANS_PER_ROTATION;
    }
}
