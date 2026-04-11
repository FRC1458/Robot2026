package frc.robot.lib.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO {
    @AutoLog
    public static class TalonFXIOInputs {
        public int id = 0;
        public boolean connected = false;
        public double positionRotations = 0.0;
        public double velocityRPS = 0.0;
        public double accelerationRPSS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public String controlRequest = "";
    }

    public final String name;
    public final TalonFX motor;
    public final TalonFXIOInputsAutoLogged inputs;

    public StatusSignal<Angle> position;
    public StatusSignal<AngularVelocity> velocity;
    public StatusSignal<AngularAcceleration> acceleration;
    public StatusSignal<Voltage> appliedVoltage;
    public StatusSignal<Voltage> supplyVoltage;
    public StatusSignal<Current> statorCurrent;
    public StatusSignal<Current> supplyCurrent;
    public StatusSignal<Temperature> temperature;
            
    private final Debouncer connectedDebouncer =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public TalonFXIO(String name, TalonFX motor) {
        this.name = name;
        this.motor = motor;
        inputs = new TalonFXIOInputsAutoLogged();
        inputs.id = motor.getDeviceID();
        position = motor.getPosition();
        velocity = motor.getVelocity();
        acceleration = motor.getAcceleration();
        appliedVoltage = motor.getMotorVoltage();
        supplyVoltage = motor.getSupplyVoltage();
        statorCurrent = motor.getStatorCurrent();
        supplyCurrent = motor.getSupplyCurrent();
        temperature = motor.getDeviceTemp();
    }

    public void updateInputs() {
        var status = BaseStatusSignal.refreshAll(
            position,
            velocity,
            appliedVoltage,
            supplyVoltage,
            statorCurrent,
            supplyCurrent,
            temperature);
        inputs.connected = connectedDebouncer.calculate(status.isOK());
        inputs.positionRotations = position.getValueAsDouble();
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.accelerationRPSS = acceleration.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.supplyVolts = supplyVoltage.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.temperatureCelsius = temperature.getValueAsDouble();
        inputs.controlRequest = motor.getAppliedControl().getName();
    }

    public void process() {
        Logger.processInputs(name, inputs);
    }
}