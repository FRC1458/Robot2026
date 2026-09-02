package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {
    public static final int ROLLER_ID = 33;
    public static final int L_MOTOR_ID = 26;
    public static final int R_MOTOR_ID = 21;
    public static final AngularVelocity FEEDER_SPEED = RotationsPerSecond.of(30);
    public static final AngularVelocity ROLLER_SPEED = RotationsPerSecond.of(15);

    public static final TalonFXConfiguration L_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKV(0.0)
                .withKP(0.3)
                .withKI(0.0)
                .withKD(0.0)
                .withKA(0.021119)
                .withKS(0.69736)
                .withKV(0.10261)) 
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0));
    
    public static final TalonFXConfiguration R_CONFIG = L_CONFIG.clone();
    
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
            .withMotorOutput(new MotorOutputConfigs().withInverted(
                InvertedValue.CounterClockwise_Positive));        
}