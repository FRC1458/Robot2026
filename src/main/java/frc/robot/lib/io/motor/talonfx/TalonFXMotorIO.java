package frc.robot.lib.io.motor.talonfx;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.motor.MotorIO;
import frc.robot.lib.util.CanDevice;
import frc.robot.lib.util.CtreUtil;

public class TalonFXMotorIO implements MotorIO<TalonFXInputs> {
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
    private final VoltageOut voltageControl = new VoltageOut(0.0);
    private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
    private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);
    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
        new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
    private final CoastOut coastControl = new CoastOut();
    private final StaticBrake brakeControl = new StaticBrake();

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentStatorSignal;
    private final StatusSignal<Current> currentSupplySignal;
    private final StatusSignal<Angle> rawRotorPositionSignal;
    private final StatusSignal<Temperature> temperatureSignal;

    private final List<BaseStatusSignal> signals;
    @AutoLogOutput
    private ControlRequest request = coastControl;
    private TalonFX motor;

    private final String name;

    public TalonFXMotorIO(CanDevice device, TalonFXConfiguration config, String name) {
        this.name = name;
        motor = new TalonFX(device.id(), device.bus());
        CtreUtil.applyConfiguration(motor, config);

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        currentStatorSignal = motor.getStatorCurrent();
        currentSupplySignal = motor.getSupplyCurrent();
        rawRotorPositionSignal = motor.getRotorPosition();
        temperatureSignal = motor.getDeviceTemp();

        signals = List.of(
            positionSignal, 
            velocitySignal, 
            voltageSignal,
            currentStatorSignal, 
            currentSupplySignal, 
            rawRotorPositionSignal,
            temperatureSignal);
    }

    @Override
    public void readInputs(TalonFXInputs inputs) {
        BaseStatusSignal.refreshAll(signals);

        inputs.position.mut_replace(positionSignal.getValue());
        inputs.velocity.mut_replace(velocitySignal.getValue());
        inputs.appliedVoltage.mut_replace(voltageSignal.getValue());
        inputs.statorCurrent.mut_replace(currentStatorSignal.getValue());
        inputs.supplyCurrent.mut_replace(currentSupplySignal.getValue());
        inputs.rawRotorPosition.mut_replace(rawRotorPositionSignal.getValue());
        inputs.temperature.mut_replace(temperatureSignal.getValue());
    }

    public StatusCode setControl(ControlRequest request) {
        this.request = request;
        Logger.recordOutput(name + "/ControlRequest/Type", request.getName());
        return motor.setControl(request);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        Logger.recordOutput(name + "/ControlRequest/DutyCycle", dutyCycle);
        setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        Logger.recordOutput(name + "/ControlRequest/Voltage", voltage);
        setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void setProfiledSetpoint(Angle position, AngularVelocity velocity, AngularAcceleration acceleration, int slot, Voltage feedforward) {
        Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/Position", position);
        Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/Velocity", velocity);
        Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/Acceleration", acceleration);
        Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/slot", slot);
        Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/Feedforward", feedforward);
        setControl(dynamicMotionMagicVoltage
            .withPosition(position)
            .withVelocity(velocity)
            .withAcceleration(acceleration)
            .withSlot(slot)
            .withFeedForward(feedforward));
    }

    @Override
    public void setBrake() {
        setControl(brakeControl);
    }

    @Override
    public void setCoast() {
        setControl(coastControl);
    }

    @Override
    public void setPosition(Angle position) {
        Logger.recordOutput(name + "/ControlRequest/Position", position);
        setControl(positionVoltageControl.withPosition(position));
    }

    @Override
    public void setProfiledPosition(Angle position) {
        Logger.recordOutput(name + "/ControlRequest/ProfiledPosition", position);
        setControl(motionMagicPositionControl.withPosition(position));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        Logger.recordOutput(name + "/ControlRequest/Velocity", velocity);
        setControl(velocityVoltageControl.withVelocity(velocity));
    }

    @Override
    public void setTorqueCurrent(Current current) {
        Logger.recordOutput(name + "/ControlRequest/TorqueCurrent", current);
        setControl(torqueCurrentFOC.withOutput(current));
    }

    @Override
    public void resetPosition(Angle position) {
        CtreUtil.tryUntilOK(() -> motor.setPosition(position), motor.getDeviceID());
        Logger.recordOutput(name + "/LastResetPosition", position);
    }

    @Override
    public String getName() {
        return name;
    }
}
