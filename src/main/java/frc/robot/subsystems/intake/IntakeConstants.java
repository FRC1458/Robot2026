package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
public class IntakeConstants {
    public static final double PIVOT_GEAR_RATIO = 44.0 / 18.0 * 5.0 * 4.0;
    public static final Mass INTAKE_MASS = Kilograms.of(3.656684786); // kg, ideally
    public static final Distance INTAKE_LENGTH = Meters.of(0.1746631508); //m, hopefully
    
    public static final Angle PIVOT_POS_MIN = Degrees.of(0);
    public static final Angle PIVOT_POS_MAX = Degrees.of(127);
    public static final Angle PIVOT_POS_UP = Degrees.of(110);
    public static final Angle PIVOT_POS_MID = Degrees.of(40);
    public static final Angle PIVOT_EPSILON = Degrees.of(10);

    public static final AngularVelocity ROLLER_SPEED = RotationsPerSecond.of(30);
    
    public static final AngularVelocity OUTTAKE_SPEED = RotationsPerSecond.of(-30);
    
    public static final int ROLLER_ID = 31;
    public static final int PIVOT_ID = 32;

    public static final TalonFXConfiguration ROLLER_CONFIG = 
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
            .withPeakForwardVoltage(12.0)
            .withPeakReverseVoltage(-12.0))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive));        
    
    
    public static final TalonFXConfiguration PIVOT_CONFIG = 
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(20.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKG(0.0)
                .withGravityType(GravityTypeValue.Arm_Cosine))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(10)
                .withMotionMagicCruiseVelocity(10)
                .withMotionMagicJerk(16))
            .withVoltage(new VoltageConfigs()
            .withPeakForwardVoltage(12.0)
            .withPeakReverseVoltage(-12.0))
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(PIVOT_GEAR_RATIO));        
    
    
}
