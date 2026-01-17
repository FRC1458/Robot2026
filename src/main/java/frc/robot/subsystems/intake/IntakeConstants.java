package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class IntakeConstants {
    public static final double INTAKE_SPEED = 20; //rotations per second?
    // public static final double BAR_VOLTAGE = 4.0; //?
    public static final double BAR_POSITION_DOWN = 0.30;
    public static final double BAR_POSITION_UP = 0.0;
    public static final double BAR_GEAR_RATIO = 1.0;
    public static final double BAR_POS_MIN = 0.0;
    public static final double BAR_POS_MAX = 0.30;
    //etc

    public static enum Motors { //TODO: set motor ids; use separate file for ports?
        WHEEL(-1),
        BAR(-1);
        public final int id;
        private Motors(int id) {
            this.id = id;
        }
    }

    // public static enum Lasers { //TODO: set laser ids
    //     LEFT(-1),
    //     RIGHT(-1);
    //     public final int id;
    //     private Lasers(int id) {
    //         this.id = id;
    //     }
    // }


    public static TalonFXConfiguration getWheelConfig() { //TODO: values
        return new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKV(0.0)
            .withKP(0.3)
            .withKI(0.0)
            .withKD(0.0))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(30)
            .withSupplyCurrentLimit(30))
        .withVoltage(new VoltageConfigs()
        .withPeakForwardVoltage(12.0)
        .withPeakReverseVoltage(-12.0));        
    }
    

    public static TalonFXConfiguration getBarConfig() { //TODO: values
        return new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKV(0.0)
            .withKP(0.3)
            .withKI(0.0)
            .withKD(0.0)
            .withKG(0.0).withGravityType(GravityTypeValue.Arm_Cosine))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(30)
            .withSupplyCurrentLimit(30))
        .withVoltage(new VoltageConfigs()
        .withPeakForwardVoltage(12.0)
        .withPeakReverseVoltage(-12.0));        
    }
    
}
