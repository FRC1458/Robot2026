package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;

public class ControlsMapping {
	public static void bind() {
		Robot.controller
				.rightBumper()
				.onTrue(Commands.print("Hi"))
				.whileTrue(
						Commands.parallel(
								Robot.getInstance().drive.headingLockToHub(),
								Robot.getInstance()
										.drive
										.waitUntilAligned()
										.andThen(
												Commands.parallel(
														Robot.getInstance()
																.shooter
																.shootAll(Robot.getInstance().drive::getDistanceToHub),
														Robot.getInstance().intake.agitate(),
														Robot.getInstance().indexer.indexAll()))));

		Robot.controller
				.leftBumper()
				.onTrue(Commands.print("Hi"))
				.whileTrue(Robot.getInstance().intake.intake());
	}
}
