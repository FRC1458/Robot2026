package frc.robot.lib.io;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CancoderIO {
    @AutoLog
    public static class CancoderIOInputs {
        public int id = 0;
        public boolean connected = false;
        public double positionRotations = 0.0;
        public double velocityRPS = 0.0;
    }

    public final String name;
    public final CANcoder motor;
    public final CancoderIOInputsAutoLogged inputs;

    public StatusSignal<Angle> position;
    public StatusSignal<AngularVelocity> velocity;
            
    private final Debouncer connectedDebouncer =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public CancoderIO(String name, CANcoder motor) {
        this.name = name;
        this.motor = motor;
        inputs = new CancoderIOInputsAutoLogged();
        inputs.id = motor.getDeviceID();
        position = motor.getPosition();
        velocity = motor.getVelocity();
    }

    public void updateInputs() {
        var status = BaseStatusSignal.refreshAll(
            position,
            velocity);
        inputs.connected = connectedDebouncer.calculate(status.isOK());
        inputs.positionRotations = position.getValueAsDouble();
        inputs.velocityRPS = velocity.getValueAsDouble();
    }

    public void process() {
        Logger.processInputs(name, inputs);
    }
}
