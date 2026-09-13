package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.concurrent.atomic.AtomicBoolean;

public class ControlsMapping {
	private static final AtomicBoolean switcher = new AtomicBoolean(false);

	public static void bind() {
		final Robot robot = Robot.getInstance();
		final Drive drive = robot.drive;
		final Intake intake = robot.intake;
		final Indexer indexer = robot.indexer;
		final Shooter shooter = robot.shooter;

		Robot.controller
				.y()
				.whileTrue(
						Commands.parallel(
										drive.headingLockToHub(),
										drive
												.waitUntilAligned()
												.asProxy()
												.andThen(
														Commands.parallel(
																shooter.shootAll(drive::getDistanceToHub),
																Commands.sequence(
																		shooter.waitForAll().asProxy(),
																		Commands.parallel(intake.agitate(), indexer.indexAll())))))
								.withName("shoot"));

		Robot.controller
				.y()
				.whileTrue(
						Commands.parallel(
										drive.headingLockToHub(),
										Commands.parallel(
												drive.waitUntilAligned().asProxy(),
												shooter.shootAll(drive::getDistanceToHub),
												Commands.sequence(
														shooter.waitForAll().asProxy(),
														Commands.parallel(intake.agitate(), indexer.indexAll()))))
								.withName("shoot"));

		Robot.controller.rightBumper().whileTrue(intake.intake().withName("intake"));

		Robot.controller.rightTrigger().whileTrue(intake.outtake().withName("outtake"));

		Robot.controller
				.a()
				.whileTrue(
						Commands.parallel(
										drive.passAlign(),
										drive
												.waitUntilAlignedPass()
												.asProxy()
												.andThen(
														Commands.parallel(
																shooter.pass(),
																Commands.sequence(
																		shooter.waitForAll().asProxy(),
																		Commands.parallel(intake.agitate(), indexer.indexAll())))))
								.withName("pass"));
		Robot.controller.povDown().whileTrue(intake.calibrate());

		Robot.controller
				.b()
				.whileTrue(
						Commands.runOnce(() -> switcher.set(!switcher.get()))
								.andThen(
										Commands.either(
												intake
														.lower()
														.alongWith(
																Commands.runOnce(() -> intake.setDefaultCommand(intake.lower()))),
												intake
														.raise()
														.alongWith(
																Commands.runOnce(() -> intake.setDefaultCommand(intake.raise()))),
												switcher::get)));
	}
}
