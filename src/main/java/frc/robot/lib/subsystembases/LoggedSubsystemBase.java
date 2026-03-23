package frc.robot.lib.subsystembases;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.io.IO;
import frc.robot.lib.io.Inputs;

public class LoggedSubsystemBase<
        I extends Inputs, 
        A extends Inputs & LoggableInputs, 
        O extends IO<I>, 
        C extends SubsystemConstants> 
    extends SubsystemBase 
{
    protected A inputs;
    protected O io;
    protected C constants;

    public LoggedSubsystemBase(C constants) {
        this.constants = constants;
    }

    private MutTime latency = Seconds.mutable(0.0);

    @Override
    public void periodic() {
        double timestamp = Timer.getFPGATimestamp();
        io.readInputs((I) inputs); // java warcrime
        Logger.processInputs(getName(), inputs);
        Logger.recordOutput(
            getName() + "/currentCommand",
            (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
        latency.mut_setMagnitude(Timer.getFPGATimestamp() - timestamp);
        Logger.recordOutput(
            getName() + "/latency", latency);
    }
}
