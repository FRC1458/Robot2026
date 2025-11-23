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
    public static final double EPSILON = 0.01;
    
    public static final double SPROCKET_RADIUS = Units.inchesToMeters(0.8785);
    public static final double SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS * Constants.TAU;
    public static final double GEAR_RATIO = 9;

    private static final double RADIANCONVERSION = 180.0/Math.PI;

    public static final double MAX_ANGLE = 90.0 * RADIANCONVERSION;
    public static final double MIN_ANGLE = 0.0 * RADIANCONVERSION;

    public static enum Motors {
        PIVOT_MOTOR(67);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }


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
                .withStatorCurrentLimit(40.0)
                .withSupplyCurrentLimit(40.0))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0));
    }

    public static double angleToRotations(double angle) {
        return angle / 360.0 * GEAR_RATIO * RADIANCONVERSION;
    }

    public static double rotationsToAngle(double rotations) {
        return rotations / GEAR_RATIO * 360.0 * RADIANCONVERSION;
    }
}
