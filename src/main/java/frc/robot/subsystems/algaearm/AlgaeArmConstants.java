package frc.robot.subsystems.algaearm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class AlgaeArmConstants {
    public static final double EPSILON = Constants.TAU / 360 ; // Radians
    public static final double GEAR_RATIO = 24;

    private static final double RADIANS_PER_ROTATION = Constants.TAU / GEAR_RATIO; // Approximated via counting both gears used + the gear box. gear box is 2:1

    public static final double MAX_ANGLE = Math.PI / 2;
    public static final double MIN_ANGLE = 0; // Min angle in Radians
    public static final double LENGTH = 0.43; // Meters


    public static enum Angles {
        STOW(0.0),
        DEPLOY(Math.PI / 2);
        public final double angle;
        private Angles(double angle) {
            this.angle = angle;
        }
    }

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
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs()
                .withKS(0.125)
                .withKV(0.0)
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.05)
                .withKG(0.5)
                .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(172.5)
                .withMotionMagicCruiseVelocity(30.0)
                .withMotionMagicJerk(1600.0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30.0)
                .withSupplyCurrentLimit(30.0))
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
