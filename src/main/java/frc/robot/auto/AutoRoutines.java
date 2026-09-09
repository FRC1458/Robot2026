package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.auto.AutoSelector.Auto;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import frc.robot.lib.trajectory.TrajectoryLoader;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public final class AutoRoutines {
	@Auto(name = "Pid Test")
	public static Command testPidToPose() {
		final Robot robot = Robot.getInstance();
		final Drive drive = robot.drive;
		return drive.autoAlign(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
	}

	@Auto(name = "Trajectory Test")
	public static Command testTrajectoryAuto() {
		final Robot robot = Robot.getInstance();
		final Drive drive = robot.drive;
		RedTrajectory traj =
				TrajectoryLoader.loadAutoTrajectory(TrajectoryType.CHOREO, "testPath3").get();
		return Robot.getInstance().drive.trajectory(traj);
	}

	@Auto(name = "right neutral auto")
	public static Command rightAutoNeutral() {
		final Robot robot = Robot.getInstance();
		final Drive drive = robot.drive;
		final Intake intake = robot.intake;
		final Indexer indexer = robot.indexer;
		final Shooter shooter = robot.shooter;

		var tTrenchRight =
				TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, "TrenchRight");

		if (tTrenchRight.isEmpty()) {
			DriverStation.reportWarning("Something happened", true);
			return Commands.none();
		}

		var tSwipeRight = TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, "SwipeRight");

		if (tSwipeRight.isEmpty()) {
			DriverStation.reportWarning("Something happened", true);
			return Commands.none();
		}

		var tReturnTrenchRight =
				TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, "ReturnTrenchRight");

		if (tReturnTrenchRight.isEmpty()) {
			DriverStation.reportWarning("Something happened", true);
			return Commands.none();
		}
		var tReturnBumpRight =
				TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, "ReturnBumpRight");

		if (tReturnBumpRight.isEmpty()) {
			DriverStation.reportWarning("Something happened", true);
			return Commands.none();
		}

		var tBackToNeutralRight =
				TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, "BackToNeutralRight");

		if (tBackToNeutralRight.isEmpty()) {
			DriverStation.reportWarning("Something happened", true);
			return Commands.none();
		}

		var crossTrench = tTrenchRight.get();
		var swipe = tSwipeRight.get();
		var back = tReturnBumpRight.get();
		var backToNeutral = tBackToNeutralRight.get();
		return Commands.sequence(
				Commands.print(Timer.getFPGATimestamp() + ": Time start"),
				Commands.race(intake.calibrate(), drive.trajectory(crossTrench)),
				Commands.race(drive.trajectory(swipe), intake.intake()),
				drive.trajectory(back),
				Commands.parallel(
								drive.headingLockToHub(),
								Commands.sequence(
										drive.waitUntilAligned().asProxy(),
										Commands.parallel(
												intake.agitate(),
												indexer.indexAll(),
												shooter.shootAll(drive::getDistanceToHub))))
						.raceWith(Commands.waitSeconds(4)),
				drive.trajectory(backToNeutral),
				drive.trajectory(crossTrench));

		// .andThen(Robot.getInstance().intake.calibrate())
		// 	.alongWith(
		// 		Robot.getInstance().drive.trajectory(crossTrench))
		// .andThen(
		// 	Robot.getInstance().intake.intake())
		// .andThen(
		// 	Robot.getInstance().drive.trajectory(swipe))
		// .andThen(
		// 	Robot.getInstance().intake.lower()
		// 		.alongWith(
		// 			Robot.getInstance().drive.trajectory(back)))
		// .andThen(
		// 	Robot.getInstance().drive.headingLockToHub()
		// 		.alongWith(
		// 			Robot.getInstance().drive.waitUntilAligned().asProxy().andThen(
		// 				Robot.getInstance().shooter.shootAll(Robot.getInstance().drive::getDistanceToHub)
		// 					.andThen(
		// 						Commands.waitSeconds(0.5))
		// 					.andThen(
		// 						Robot.getInstance().indexer.indexAll())
		// 					.andThen(
		// 						Robot.getInstance().intake.agitate())
		// 					.andThen(
		// 						Commands.waitSeconds(3))))
		// 		.raceWith(
		// 			Commands.waitSeconds(4)))
		// .andThen(
		// 	Robot.getInstance().drive.trajectory(backToNeutral))
		// .andThen(
		// 	Robot.getInstance().drive.trajectory(crossTrench))
		// .andThen(
		// 	Robot.getInstance().intake.intake())
		// .andThen(
		// 	Robot.getInstance().drive.trajectory(swipe))
		// .andThen(
		// 	Robot.getInstance().intake.lower()
		// 		.alongWith(
		// 			Robot.getInstance().drive.trajectory(back)))
		// .andThen(
		// 	Robot.getInstance().drive.headingLockToHub()
		// 		.raceWith(
		// 			Robot.getInstance().shooter.shootAll(Robot.getInstance().drive::getDistanceToHub)
		// 				.andThen(
		// 					Commands.waitSeconds(0.5))
		// 				.andThen(
		// 					Robot.getInstance().indexer.indexAll())
		// 				.andThen(
		// 					Robot.getInstance().intake.agitate())
		// 				.andThen(
		// 					Commands.waitSeconds(3))))
		// .andThen(
		// 	Commands.print(Timer.getFPGATimestamp() + ": Time end"),
		// 	Commands.idle());
	}

	// @Auto(name = "left neutral auto")
	// public static Command leftAutoNeutral() {
	// 	var tTrenchLeft = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"TrenchLeft");

	// 	if (tTrenchLeft.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var tSwipeLeft = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"SwipeLeft");

	// 	if (tSwipeLeft.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var tReturnTrenchLeft = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"ReturnTrenchLeft");

	// 	if (tReturnTrenchLeft.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var crossTrench = tTrenchLeft.get();
	// 	var swipe = tSwipeLeft.get();
	// 	var back = tReturnTrenchLeft.get();
	// 	return Commands.print(Timer.getFPGATimestamp() + ": Time start")
	// 		.andThen(Robot.getInstance().intake.calibrate())
	// 		.alongWith(
	// 			Robot.getInstance().drive.trajectory(crossTrench))
	// 		.andThen(
	// 			Robot.getInstance().intake.intake())
	// 		.andThen(
	// 			Robot.getInstance().drive.trajectory(swipe))
	// 		.andThen(
	// 			Robot.getInstance().intake.lower()
	// 				.alongWith(
	// 					Robot.getInstance().drive.trajectory(back)))
	// 		.andThen(
	// 			Robot.getInstance().drive.headingLockToHub()
	// 				.raceWith(
	// 					Robot.getInstance().shooter.shootAll(Robot.getInstance().drive::getDistanceToHub)
	// 						.andThen(
	// 							Commands.waitSeconds(0.5))
	// 						.andThen(
	// 							Robot.getInstance().indexer.indexAll())
	// 						.andThen(
	// 							Robot.getInstance().intake.agitate())
	// 						.andThen(
	// 							Commands.waitSeconds(3))))
	// 		.andThen(
	// 			Commands.print(Timer.getFPGATimestamp() + ": Time end"),
	// 			Commands.idle())
	// 		.finallyDo(
	// 			() -> {
	// 				CommandScheduler.getInstance().schedule(
	// 					Robot.getInstance().drive.openLoopControl(),
	// 					Robot.getInstance().intake.lower(),
	// 					Automation.stopShoot(),
	// 					Automation.stopIndex());
	// 			});
	// }

	// @Auto(name = "center auto")
	// public static Command centerAuto() {
	// 	var tDepotCenter = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"DepotCenter");

	// 	if (tDepotCenter.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var tDepotShootCenter = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"DepotShootCenter");

	// 	if (tDepotShootCenter.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var tStationCenter = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"StationCenter");

	// 	if (tStationCenter.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var tStationShootCenter = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"StationShootCenter");

	// 	if (tStationCenter.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var depotCenter = tDepotCenter.get();
	// 	var depotShootCenter = tDepotShootCenter.get();
	// 	var stationCenter = tStationCenter.get();
	// 	var stationShootCenter = tStationShootCenter.get();

	// 	return Commands.print(Timer.getFPGATimestamp() + ": Time start")
	// 		.andThen(Robot.getInstance().intake.calibrateZero())
	// 		.andThen(Robot.getInstance().intake.intake())
	// 		.alongWith(Robot.getInstance().drive.trajectory(depotCenter))
	// 		.andThen(
	// 			Robot.getInstance().drive.trajectory(depotShootCenter))
	// 		.andThen(
	// 			Robot.getInstance().drive.headingLockToHub()
	// 				.raceWith(
	// 					Robot.getInstance().shooter.shootAll(Robot.getInstance().drive::getDistanceToHub)
	// 						.andThen(
	// 							Commands.waitSeconds(0.5))
	// 						.andThen(
	// 							Robot.getInstance().indexer.indexAll())
	// 						.andThen(
	// 							Commands.waitSeconds(3))))
	// 		.andThen(
	// 			Robot.getInstance().intake.lower(),
	// 			Automation.stopShoot(),
	// 			Automation.stopIndex())
	// 		.andThen(
	// 			Robot.getInstance().drive.trajectory(stationCenter))
	// 		.andThen(
	// 			Commands.waitSeconds(3))
	// 		.andThen(
	// 			Robot.getInstance().drive.trajectory(stationShootCenter))
	// 		.andThen(
	// 			Robot.getInstance().drive.headingLockToHub()
	// 				.alongWith(
	// 					Robot.getInstance().shooter.shootAll(Robot.getInstance().drive::getDistanceToHub)
	// 						.andThen(
	// 							Commands.waitSeconds(0.5))
	// 						.andThen(
	// 							Robot.getInstance().indexer.indexAll())
	// 						.andThen(
	// 							Commands.waitSeconds(3))
	// 						.andThen(
	// 							Robot.getInstance().intake.stow())))
	// 		.andThen(
	// 			Commands.print(Timer.getFPGATimestamp() + ": Time end"),
	// 			Commands.idle())
	// 		.finallyDo(
	// 			() -> {
	// 				CommandScheduler.getInstance().schedule(
	// 					Robot.getInstance().drive.openLoopControl(),
	// 					Robot.getInstance().intake.lower(),
	// 					Automation.stopShoot(),
	// 					Automation.stopIndex());
	// 			});
	// }
}
