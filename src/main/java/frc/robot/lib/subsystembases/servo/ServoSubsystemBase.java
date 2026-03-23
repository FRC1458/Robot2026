package frc.robot.lib.subsystembases.servo;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.io.motor.MotorIO;
import frc.robot.lib.io.motor.MotorInputs;
import frc.robot.lib.subsystembases.LoggedSubsystemBase;
import frc.robot.lib.util.Util;

public class ServoSubsystemBase<
        I extends MotorInputs, 
        A extends MotorInputs & LoggableInputs, 
        O extends MotorIO<I>> 
    extends LoggedSubsystemBase<I, A, O, ServoSubsystemConstants>
{
    public ServoSubsystemBase(ServoSubsystemConstants constants) {
        super(constants);
    }

    public Command followProfiledTrajectory(
        Supplier<Angle> positionSupplier,
        Supplier<AngularVelocity> velocitySupplier,
        Supplier<AngularAcceleration> accelerationSupplier,
        Supplier<Voltage> feedforwardSupplier
    ) {
        return run(
                () ->
                    io.setProfiledSetpoint(
                        positionSupplier.get(), 
                        velocitySupplier.get(), 
                        accelerationSupplier.get(), 
                        feedforwardSupplier.get()))
            .andThen(idle());
    }

    public Command moveToPositionBlocking(Angle position) {
        return runOnce(() -> io.setPosition(position)).until(isPositionInRange(position));
    }

    public Command profiledMoveToPositionBlocking(Angle position) {
        return runOnce(() -> io.setProfiledPosition(position)).until(isPositionInRange(position));
    }

    public Command moveWithVelocityBlocking(AngularVelocity velocity) {
        return runOnce(() -> io.setVelocity(velocity)).until(isVelocityInRange(velocity));
    }

    public Command moveToPosition(Angle position) {
        return runOnce(() -> io.setPosition(position)).andThen(idle());
    }

    public Command profiledMoveToPosition(Angle position) {
        return runOnce(() -> io.setProfiledPosition(position)).andThen(idle());
    }

    public Command moveWithVelocity(AngularVelocity velocity) {
        return runOnce(() -> io.setVelocity(velocity)).andThen(idle());
    }

    public Command moveWithCurrent(Current current) {
        return runOnce(() -> io.setTorqueCurrent(current)).andThen(idle());
    }

    public Command brake() {
        return runOnce(() -> io.setBrake()).andThen(idle());
    }

    public Command coast() {
        return runOnce(() -> io.setCoast()).andThen(idle());
    }

    public void resetPosition(Angle position) {
        io.resetPosition(position);
    }

    public Trigger isPositionInRange(Angle target) {
        return new Trigger(
                () -> Util.epsilonEquals(
                    inputs.position, 
                    target, 
                    constants.positionEpsilon))
            .debounce(constants.debouncePeriod.in(Seconds));
    }
    
    public Trigger isVelocityInRange(AngularVelocity target) {
        return new Trigger(
                () -> Util.epsilonEquals(
                    inputs.velocity, 
                    target, 
                    constants.velocityEpsilon))
            .debounce(constants.debouncePeriod.in(Seconds));
    }

}
