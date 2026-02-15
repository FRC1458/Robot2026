package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimbConstants {

    public static final double EPSILON = 0.03; // Meters

    public static final double SPROCKET_RADIUS = 0.0412; // Effective pitch radius
    public static final double GEAR_RATIO = 9;
    public static final double CONVERSION_FACTOR = 67;

    public static final double SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS * Constants.TAU;
    public static final double END_EFFECTOR_HEIGHT = 0.0; // Meters // CHANGE THIS
    public static final double METERS_PER_ROTATION = 0.028776; // Approximated using measurement
    public static final double CARRIAGE_WEIGHT = 7.55; // kg

    // Yo uu hav eto tTune all of this above me

    public static final double MAX_ACCEL = 1.5;
    public static final double MAX_SPEED = 1.0; // m/s
    
    public static final int CLIMB_MOTOR_ID = 25; // change this

    public static enum Setpoint {
        BASE(0.003), // small offset to prevent stalling (allegedly)
        UP(Units.inchesToMeters(5.0));
        public final double height;
        private Setpoint(double height) {
            this.height = height;
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
                .withKG(0.375))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(MAX_ACCEL) // 1.0 m/s^2
                .withMotionMagicCruiseVelocity(MAX_SPEED)
                .withMotionMagicJerk(320))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimit(60))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0))
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(CONVERSION_FACTOR));
    }
}
