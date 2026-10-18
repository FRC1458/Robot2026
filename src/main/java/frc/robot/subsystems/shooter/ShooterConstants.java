package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.lib.util.Util.InchSqPounds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.lib.util.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
	public static final int HOOD_ID = 57; // TODO: add
	public static final int BR_ID = 22;
	public static final int TR_ID = 23;
	public static final int TL_ID = 24;
	public static final int BL_ID = 25;

	public static final AngularVelocity PASSING_SPEED = RotationsPerSecond.of(75);
	public static final MomentOfInertia MOI = InchSqPounds.of(4.619883);

	public static final AngularVelocity EPS = RotationsPerSecond.of(5);
	public static final Angle HOOD_EPS = Degrees.of(2.0);
	public static final double RPS = 30.0;

	public static final Translation3d LEFT_OFFSET =
			new Translation3d(Inches.of(7.300000), Inches.of(8.562500), Inches.of(15.829364));
	public static final Translation3d RIGHT_OFFSET =
			new Translation3d(Inches.of(7.300000), Inches.of(-8.562500), Inches.of(15.829364));


	public static final Translation3d OFFSET =
			new Translation3d(Inches.of(7.300000), Inches.of(0), Inches.of(15.829364));

	public static final Mass HOOD_MASS = Pounds.of(7.0);
	public static final Distance HOOD_LENGTH = Inches.of(7.0);
	public static final MomentOfInertia HOOD_MOI = InchSqPounds.of(500.0);
	public static final Angle HOOD_POS_MIN = Degrees.of(15);
	public static final Angle HOOD_POS_MAX = Degrees.of(45);
	public static final double HOOD_GEAR_RATIO = 190.0 / 15.0 * 70.0 / 24.0;

	public static final Rotation3d rotation = new Rotation3d(0, -77.5 / 180.0 * Math.PI, 0);

	public static final InterpolatingDoubleTreeMap VEL_MAP = new InterpolatingDoubleTreeMap();
	public static final InterpolatingDoubleTreeMap ANGLE_MAP = new InterpolatingDoubleTreeMap();
	public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

	static {
		// Very un scuffed placeholder values
		VEL_MAP.put(Double.MIN_VALUE, 30.250);
		VEL_MAP.put(1.700, 30.917);
		VEL_MAP.put(2.000, 31.250);
		VEL_MAP.put(2.215, 32.333);
		VEL_MAP.put(2.495, 33.583);
		VEL_MAP.put(2.888, 34.417);
		VEL_MAP.put(3.222, 35.500);
		VEL_MAP.put(3.570, 36.667);
		VEL_MAP.put(3.830, 37.333);
		VEL_MAP.put(4.150, 39.000);
		VEL_MAP.put(4.750, 39.417);
		VEL_MAP.put(Double.MAX_VALUE, 40.167);

		ANGLE_MAP.put(Double.MIN_VALUE, 17.5);
		ANGLE_MAP.put(1.700, 17.5);
		ANGLE_MAP.put(2.000, 18.5);
		ANGLE_MAP.put(2.215, 19.0);
		ANGLE_MAP.put(2.495, 19.5);
		ANGLE_MAP.put(2.888, 20.5);
		ANGLE_MAP.put(3.222, 21.5);
		ANGLE_MAP.put(3.570, 22.5);
		ANGLE_MAP.put(3.830, 23.0);
		ANGLE_MAP.put(4.150, 24.5);
		ANGLE_MAP.put(4.750, 26.5);
		ANGLE_MAP.put(Double.MAX_VALUE, 27.0);

		TOF_MAP.put(1.967387, 1.046);
		TOF_MAP.put(2.207408, 1.080);
		TOF_MAP.put(2.570000, 1.100);
		TOF_MAP.put(2.663911, 1.107);
		TOF_MAP.put(2.750000, 1.200);
		TOF_MAP.put(2.806310, 1.233);
		TOF_MAP.put(3.000000, 1.237);
		TOF_MAP.put(3.569663, 1.267);
	}

	public static final TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration()
			.withSlot0(
					new Slot0Configs()
							.withKV(0.0)
							.withKP(320.0)
							.withKI(0.0)
							.withKD(0.0)
							.withKG(0.0)
							.withGravityType(GravityTypeValue.Arm_Cosine))
			.withCurrentLimits(
					new CurrentLimitsConfigs()
							.withStatorCurrentLimit(40)
							.withSupplyCurrentLimit(30)
							.withStatorCurrentLimitEnable(true)
							.withSupplyCurrentLimitEnable(true))
			// .withMotionMagic(
			// 		new MotionMagicConfigs()
			// 				.withMotionMagicAcceleration(HOOD_GEAR_RATIO * 5)
			// 				.withMotionMagicCruiseVelocity(HOOD_GEAR_RATIO * 2)
			// 				.withMotionMagicJerk(HOOD_GEAR_RATIO * 10))
			.withVoltage(
					new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
			.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HOOD_GEAR_RATIO));
			// .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
					
	public static final TalonFXConfiguration TR_CONFIG =
			new TalonFXConfiguration()
					.withSlot0(
							new Slot0Configs()
									.withKP(0.3)
									.withKI(0.0)
									.withKD(0.0)
									.withKA(0.00766)
									.withKS(0.18188)
									.withKV(0.12378))
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									.withStatorCurrentLimit(40)
									.withSupplyCurrentLimit(30)
									.withStatorCurrentLimitEnable(true)
									.withSupplyCurrentLimitEnable(true))
					.withVoltage(
							new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
					.withMotionMagic(
							new MotionMagicConfigs()
									.withMotionMagicAcceleration(120)
									.withMotionMagicCruiseVelocity(120)
									.withMotionMagicJerk(120))
					.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

	public static final TalonFXConfiguration TL_CONFIG =
			new TalonFXConfiguration()
					.withSlot0(
							new Slot0Configs()
									.withKP(0.3)
									.withKI(0.0)
									.withKD(0.0)
									.withKA(0.00766)
									.withKS(0.18188)
									.withKV(0.12378))
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									.withStatorCurrentLimit(40)
									.withSupplyCurrentLimit(30)
									.withStatorCurrentLimitEnable(true)
									.withSupplyCurrentLimitEnable(true))
					.withVoltage(
							new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
					.withMotionMagic(
							new MotionMagicConfigs()
									.withMotionMagicAcceleration(120)
									.withMotionMagicCruiseVelocity(120)
									.withMotionMagicJerk(120))
					.withMotorOutput(
							new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

	public static final TalonFXConfiguration BR_CONFIG =
			new TalonFXConfiguration()
					.withSlot0(
							new Slot0Configs()
									.withKP(0.3)
									.withKI(0.0)
									.withKD(0.0)
									.withKA(0.0077352)
									.withKS(0.29729)
									.withKV(0.12292))
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									.withStatorCurrentLimit(40)
									.withSupplyCurrentLimit(30)
									.withStatorCurrentLimitEnable(true)
									.withSupplyCurrentLimitEnable(true))
					.withVoltage(
							new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
					.withMotionMagic(
							new MotionMagicConfigs()
									.withMotionMagicAcceleration(120)
									.withMotionMagicCruiseVelocity(120)
									.withMotionMagicJerk(120))
					.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

	public static final TalonFXConfiguration BL_CONFIG =
			new TalonFXConfiguration()
					.withSlot0(
							new Slot0Configs()
									.withKP(0.3)
									.withKI(0.0)
									.withKD(0.0)
									.withKA(0.0077352)
									.withKS(0.29729)
									.withKV(0.12292))
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									.withStatorCurrentLimit(40)
									.withSupplyCurrentLimit(30)
									.withStatorCurrentLimitEnable(true)
									.withSupplyCurrentLimitEnable(true))
					.withVoltage(
							new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
					.withMotionMagic(
							new MotionMagicConfigs()
									.withMotionMagicAcceleration(120)
									.withMotionMagicCruiseVelocity(120)
									.withMotionMagicJerk(120))
					.withMotorOutput(
							new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
}
