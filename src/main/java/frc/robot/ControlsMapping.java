package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;

public class ControlsMapping {
	static AtomicBoolean switcher = new AtomicBoolean(false);

	public static void bind() {
		Robot.controller
				.y()
				.whileTrue(
						Commands.parallel(
										Robot.getInstance().drive.headingLockToHub(),
										Robot.getInstance()
												.drive
												.waitUntilAligned()
												.asProxy()
												.andThen(
														Commands.parallel(
																Robot.getInstance()
																		.shooter
																		.shootAll(Robot.getInstance().drive::getDistanceToHub),
																Robot.getInstance().intake.agitate(),
																Robot.getInstance().indexer.indexAll())))
								.withName("shoot"));

		Robot.controller
				.rightBumper()
				.whileTrue(Robot.getInstance().intake.intake().withName("intake"));

		Robot.controller
				.rightTrigger()
				.whileTrue(Robot.getInstance().intake.outtake().withName("outtake"));

		Robot.controller
				.a()
				.whileTrue(
						Commands.parallel(
										Robot.getInstance().shooter.pass(),
										Robot.getInstance().intake.agitate(),
										Robot.getInstance().indexer.indexAll())
								.withName("pass"));

		Robot.controller.povDown().whileTrue(Robot.getInstance().intake.calibrate());

		Robot.controller
				.b()
				.whileTrue(
						Commands.runOnce(() -> switcher.set(!switcher.get()))
								.andThen(
										Commands.either(
												Robot.getInstance()
														.intake
														.lower()
														.alongWith(
																Commands.runOnce(
																		() ->
																				Robot.getInstance()
																						.intake
																						.setDefaultCommand(
																								Robot.getInstance().intake.lower()))),
												Robot.getInstance()
														.intake
														.raise()
														.alongWith(
																Commands.runOnce(
																		() ->
																				Robot.getInstance()
																						.intake
																						.setDefaultCommand(
																								Robot.getInstance().intake.raise()))),
												switcher::get)));
	}
}
