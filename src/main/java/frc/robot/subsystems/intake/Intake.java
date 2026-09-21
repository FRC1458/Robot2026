package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.subsystem.LoggedSubsystem;

public class Intake extends LoggedSubsystem {
	public IntakePivot pivot;
	public IntakeRoller roller;

	public Intake(IntakePivot pivot, IntakeRoller roller) {
		super();
		this.pivot = pivot;
		this.roller = roller;
		setDefaultCommand(lower());
	}

	public Command intake() {
		return Commands.parallel(pivot.lower(), roller.intake()).andThen(idle());
	}

	public Command raise() {
		return Commands.parallel(pivot.raise(), roller.stop()).andThen(idle());
	}

	public Command lower() {
		return Commands.parallel(pivot.lower(), roller.stop()).andThen(idle());
	}

	public Command agitate() {
		return Commands.parallel(pivot.shake(), roller.intake()).andThen(idle());
	}

	public Command outtake() {
		return Commands.parallel(pivot.lower(), roller.outtake()).andThen(idle());
	}

	public Command stop() {
		return Commands.parallel(pivot.stop(), roller.stop()).andThen(idle());
	}

	public Command calibrate() {
		return Commands.parallel(pivot.calibrateZero(), roller.stop()).andThen(lower());
	}
}
