package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {		
    public static final int BR_ID = 22;
    public static final int TR_ID = 23;
    public static final int TL_ID = 24;
    public static final int BL_ID = 25;

    public static final InterpolatingDoubleTreeMap VEL_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap SPIN_MAP = new InterpolatingDoubleTreeMap();

    static {
        VEL_MAP.put(1.5, 23.0);
        VEL_MAP.put(2.0, 30.0);
        VEL_MAP.put(2.5, 38.0);
        VEL_MAP.put(3.0, 45.0);
        VEL_MAP.put(3.5, 60.0);

        SPIN_MAP.put(1.5, 0.0);
        SPIN_MAP.put(2.0, 0.0);
        SPIN_MAP.put(2.5, 0.0);
        SPIN_MAP.put(3.0, 10.0);
        SPIN_MAP.put(3.5, 20.0);
    }

    public static final TalonFXConfiguration TR_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0.3)
                .withKI(0.0)
                .withKD(0.0)
                .withKA(0.00766)
                .withKS(0.18188)
                .withKV(0.12378))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(120)
                .withMotionMagicCruiseVelocity(120)
                .withMotionMagicJerk(120));
    
    public static final TalonFXConfiguration TL_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0.3)
                .withKI(0.0)
                .withKD(0.0)
                .withKA(0.00766)
                .withKS(0.18188)
                .withKV(0.12378))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(120)
                .withMotionMagicCruiseVelocity(120)
                .withMotionMagicJerk(120));
    
    public static final TalonFXConfiguration BR_CONFIG=
         new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0.3)
                .withKI(0.0)
                .withKD(0.0)
                .withKA(0.0077352)
                .withKS(0.29729)
                .withKV(0.12292))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(120)
                .withMotionMagicCruiseVelocity(120)
                .withMotionMagicJerk(120));

    public static final TalonFXConfiguration BL_CONFIG=
         new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0.3)
                .withKI(0.0)
                .withKD(0.0)
                .withKA(0.0077352)
                .withKS(0.29729)
                .withKV(0.12292))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(120)
                .withMotionMagicCruiseVelocity(120)
                .withMotionMagicJerk(120));
}
