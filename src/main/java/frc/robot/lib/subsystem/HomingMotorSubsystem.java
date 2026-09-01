package frc.robot.lib.subsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.IMotor;
import frc.robot.lib.util.Util;

public class HomingMotorSubsystem extends LoggedSubsystem {
	protected IMotor io;

	protected HomingMotorSubsystem(IMotor io) {
		super();
		this.io = io;
	}

	protected Command home(Angle angle, Angle eps, IMotor.RunMode mode) {
		return run(() -> io.setRunMode(mode))
			.andThen(() -> io.setPosition(angle))
			.andThen(Commands.waitUntil(() -> Util.epsilonEquals(angle, io.getPosition(), eps)));
	}

    @Override
    protected void log() {
        io.log();
    }
}
