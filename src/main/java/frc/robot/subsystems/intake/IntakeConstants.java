package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class IntakeConstants {
    public static final double BAR_EPSILON = Units.Degrees.of(10).in(Units.Rotations);
    public static final double INTAKE_SPEED = 4.5 * 2 / (Constants.TAU * edu.wpi.first.math.util.Units.inchesToMeters(1));
    public static final double BAR_POSITION_DOWN = 0.00;
    // TODO: CHANGE WITH NEW PIVOT
    public static final double BAR_POSITION_MID = Degrees.of(70).in(Rotations);
    public static final double BAR_POSITION_UP = Degrees.of(110).in(Rotations);
    public static final double BAR_GEAR_RATIO = 44.0 / 18.0 * 5.0 * 4.0;
    public static final double BAR_POS_MIN = 0.0;
    public static final double BAR_POS_MAX = Degrees.of(127).in(Rotations);
    public static final double INTAKE_MASS = 3.656684786; // kg, ideally
    public static final double INTAKE_LENGTH = 0.1746631508; //m, hopefully
    
    public static enum Motors { //TODO: set motor ids; use separate file for ports?
        WHEEL(31),
        BAR(32);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }

    public static TalonFXConfiguration getWheelConfig() {
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
            .withPeakForwardVoltage(12.0)
            .withPeakReverseVoltage(-12.0))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive));        
    }
    
    public static TalonFXConfiguration getBarConfig() { 
        return new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(20.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKG(0.0).withGravityType(GravityTypeValue.Arm_Cosine))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withSupplyCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(10) // 1.0 m/s^2
                .withMotionMagicCruiseVelocity(10)
                .withMotionMagicJerk(16))
            .withVoltage(new VoltageConfigs()
            .withPeakForwardVoltage(12.0)
            .withPeakReverseVoltage(-12.0))
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(BAR_GEAR_RATIO));        
    }
    
}
