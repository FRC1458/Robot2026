package frc.robot.lib.subsystem;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.IMotor;
import frc.robot.lib.util.Util;

public class RollerMotorSubsystem extends LoggedSubsystem {
	protected IMotor io;

	protected RollerMotorSubsystem(IMotor io) {
		super();
		this.io = io;
	}

	protected Command roll(AngularVelocity angularVelocity, AngularVelocity eps, IMotor.RunMode mode) {
		return run(() -> io.setRunMode(mode))
			.andThen(() -> io.setVelocity(angularVelocity))
			.andThen(Commands.waitUntil(() -> Util.epsilonEquals(angularVelocity, io.getVelocity(), eps)));
	}

    @Override
    protected void log() {
        io.log();
    }
}
