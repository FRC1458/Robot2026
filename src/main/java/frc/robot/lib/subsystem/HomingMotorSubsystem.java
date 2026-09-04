package frc.robot.lib.subsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.IMotor;
import frc.robot.lib.util.Util;
import java.util.function.Supplier;

public class HomingMotorSubsystem extends LoggedSubsystem {
	protected IMotor io;

	protected HomingMotorSubsystem(IMotor io) {
		super();
		this.io = io;
	}

	protected HomingMotorSubsystem(Supplier<IMotor> io) {
		this(io.get());
	}

	protected Command runPos(Angle angle, Angle eps, IMotor.RunMode mode) {
		return runOnce(() -> io.setRunMode(mode))
				.andThen(runOnce(() -> io.setPosition(angle)))
				.andThen(Commands.waitUntil(() -> Util.epsilonEquals(angle, io.getPosition(), eps)));
	}

	@Override
	protected void log() {
		io.log();
	}
}
