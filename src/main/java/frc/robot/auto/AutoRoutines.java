package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.auto.AutoSelector.Auto;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import frc.robot.lib.trajectory.TrajectoryLoader;

public final class AutoRoutines {
	@Auto(name = "Pid Test")
	public static Command testPidToPose() {
		return Robot.getInstance().drive.autoAlign(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
	}

	@Auto(name = "Trajectory Test")
	public static Command testTrajectoryAuto() {
		RedTrajectory traj =
				TrajectoryLoader.loadAutoTrajectory(TrajectoryType.CHOREO, "testPath3").get();
		return Robot.getInstance().drive.trajectory(traj);
	}

	// @Auto(name = "right neutral auto")
	// public static Command rightAutoNeutral() {
	// 	var tTrenchRight = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"TrenchRight");

	// 	if (tTrenchRight.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var tSwipeRight = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"SwipeRight");

	// 	if (tSwipeRight.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var tReturnTrenchRight = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"ReturnTrenchRight");

	// 	if (tReturnTrenchRight.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var tBackToNeutralRight = TrajectoryLoader.loadAutoTrajectory(
	// 		TrajectoryType.PATHPLANNER,
	// 		"BackToNeutralRight");

	// 	if (tBackToNeutralRight.isEmpty()) {
	// 		DriverStation.reportWarning(
	// 			"Something happened", true);
	// 		return Commands.none();
	// 	}

	// 	var crossTrench = tTrenchRight.get();
	// 	var swipe = tSwipeRight.get();
	// 	var back = tReturnTrenchRight.get();
	// 	var backToNeutral = tBackToNeutralRight.get();
	// 	return Commands.print(Timer.getFPGATimestamp() + ": Time start")
	// 		.andThen(Intake.getInstance().calibrateZero())
	// 			.alongWith(
	// 				new TrajectoryCommand(crossTrench))
	// 		.andThen(
	// 			Intake.getInstance().intake())
	// 		.andThen(
	// 			new TrajectoryCommand(swipe))
	// 		.andThen(
	// 			Intake.getInstance().lower()
	// 				.alongWith(
	// 					new TrajectoryCommand(back)))
	// 		.andThen(
	// 			Drive.getInstance().headingLockToHub()
	// 				.raceWith(
	// 					Automation.shootAll()
	// 						.andThen(
	// 							Commands.waitSeconds(0.5))
	// 						.andThen(
	// 							Automation.indexAll())
	// 						.andThen(
	// 							Intake.getInstance().onShoot())
	// 						.andThen(
	// 							Commands.waitSeconds(3)))
	// 				.raceWith(
	// 					Commands.waitSeconds(4)))
	// 		.andThen(
	// 			Commands.parallel(
	// 				Intake.getInstance().lower(),
	// 				Automation.stopShoot(),
	// 				Automation.stopIndex()))
	// 		.andThen(
	// 			new TrajectoryCommand(backToNeutral))
	// 		.andThen(
	// 			new TrajectoryCommand(crossTrench))
	// 		.andThen(
	// 			Intake.getInstance().intake())
	// 		.andThen(
	// 			new TrajectoryCommand(swipe))
	// 		.andThen(
	// 			Intake.getInstance().lower()
	// 				.alongWith(
	// 					new TrajectoryCommand(back)))
	// 		.andThen(
	// 			Drive.getInstance().headingLockToHub()
	// 				.raceWith(
	// 					Automation.shootAll()
	// 						.andThen(
	// 							Commands.waitSeconds(0.5))
	// 						.andThen(
	// 							Automation.indexAll())
	// 						.andThen(
	// 							Intake.getInstance().onShoot())
	// 						.andThen(
	// 							Commands.waitSeconds(3))))
	// 		.andThen(
	// 			Commands.print(Timer.getFPGATimestamp() + ": Time end"),
	// 			Commands.idle())
	// 		.finallyDo(
	// 			() -> {
	// 				CommandScheduler.getInstance().schedule(
	// 					Drive.getInstance().openLoopControl(),
	// 					Intake.getInstance().lower(),
	// 					Automation.stopShoot(),
	// 					Automation.stopIndex());
	// 			});
	// }

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
	// 		.andThen(Intake.getInstance().calibrateZero())
	// 		.alongWith(
	// 			new TrajectoryCommand(crossTrench))
	// 		.andThen(
	// 			Intake.getInstance().intake())
	// 		.andThen(
	// 			new TrajectoryCommand(swipe))
	// 		.andThen(
	// 			Intake.getInstance().lower()
	// 				.alongWith(
	// 					new TrajectoryCommand(back)))
	// 		.andThen(
	// 			Drive.getInstance().headingLockToHub()
	// 				.raceWith(
	// 					Automation.shootAll()
	// 						.andThen(
	// 							Commands.waitSeconds(0.5))
	// 						.andThen(
	// 							Automation.indexAll())
	// 						.andThen(
	// 							Intake.getInstance().onShoot())
	// 						.andThen(
	// 							Commands.waitSeconds(3))))
	// 		.andThen(
	// 			Commands.print(Timer.getFPGATimestamp() + ": Time end"),
	// 			Commands.idle())
	// 		.finallyDo(
	// 			() -> {
	// 				CommandScheduler.getInstance().schedule(
	// 					Drive.getInstance().openLoopControl(),
	// 					Intake.getInstance().lower(),
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
	// 		.andThen(Intake.getInstance().calibrateZero())
	// 		.andThen(Intake.getInstance().intake())
	// 		.alongWith(new TrajectoryCommand(depotCenter))
	// 		.andThen(
	// 			new TrajectoryCommand(depotShootCenter))
	// 		.andThen(
	// 			Drive.getInstance().headingLockToHub()
	// 				.raceWith(
	// 					Automation.shootAll()
	// 						.andThen(
	// 							Commands.waitSeconds(0.5))
	// 						.andThen(
	// 							Automation.indexAll())
	// 						.andThen(
	// 							Commands.waitSeconds(3))))
	// 		.andThen(
	// 			Intake.getInstance().lower(),
	// 			Automation.stopShoot(),
	// 			Automation.stopIndex())
	// 		.andThen(
	// 			new TrajectoryCommand(stationCenter))
	// 		.andThen(
	// 			Commands.waitSeconds(3))
	// 		.andThen(
	// 			new TrajectoryCommand(stationShootCenter))
	// 		.andThen(
	// 			Drive.getInstance().headingLockToHub()
	// 				.alongWith(
	// 					Automation.shootAll()
	// 						.andThen(
	// 							Commands.waitSeconds(0.5))
	// 						.andThen(
	// 							Automation.indexAll())
	// 						.andThen(
	// 							Commands.waitSeconds(3))
	// 						.andThen(
	// 							Intake.getInstance().stow())))
	// 		.andThen(
	// 			Commands.print(Timer.getFPGATimestamp() + ": Time end"),
	// 			Commands.idle())
	// 		.finallyDo(
	// 			() -> {
	// 				CommandScheduler.getInstance().schedule(
	// 					Drive.getInstance().openLoopControl(),
	// 					Intake.getInstance().lower(),
	// 					Automation.stopShoot(),
	// 					Automation.stopIndex());
	// 			});
	// }
}
