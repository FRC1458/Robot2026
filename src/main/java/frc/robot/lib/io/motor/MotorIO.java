package frc.robot.lib.io.motor;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.io.IO;

public interface MotorIO<I extends MotorInputs> extends IO<I> {
    String getName();

    void readInputs(I inputs);

    void setDutyCycle(double dutyCycle);

    void setVoltage(Voltage volts);

    void setProfiledSetpoint(Angle position, AngularVelocity velocity, AngularAcceleration acceleration, int slot, Voltage feedforward);

    default void setProfiledSetpoint(Angle position, AngularVelocity velocity, AngularAcceleration acceleration, Voltage feedforward) {
        setProfiledSetpoint(position, velocity, acceleration, 0, feedforward);
    }

    default void setProfiledSetpoint(Angle position, AngularVelocity velocity, AngularAcceleration acceleration) {
        setProfiledSetpoint(position, velocity, acceleration, Volts.zero());
    }
    
    default void setProfiledSetpoint(Angle position, AngularVelocity velocity) {
        setProfiledSetpoint(position, velocity, RotationsPerSecondPerSecond.zero());
    }

    default void setProfiledSetpoint(Angle position) {
        setProfiledSetpoint(position, RotationsPerSecond.zero());
    }

    void setBrake();
    
    void setCoast();

    void setPosition(Angle position);

    void setProfiledPosition(Angle position);

    void setVelocity(AngularVelocity velocity);

    void setTorqueCurrent(Current current);

    void resetPosition(Angle position);
}
