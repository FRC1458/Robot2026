package frc.robot.auto;

import frc.robot.Constants;
import frc.robot.auto.AutoSelector.Auto;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.TrajectoryLoader;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.AutopilotCommand;
import frc.robot.subsystems.drive.commands.HeadingLockToHub2;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
import frc.robot.subsystems.intake.Intake;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public final class AutoRoutines {
	// @Auto(name = "Pid Test")
	public static Command testPidToPose() {
		return new PIDToPoseCommand(
			new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(120)));
	}

	// @Auto(name = "Trajectory Test")
	public static Command testTrajectoryAuto() {
		RedTrajectory traj = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.CHOREO, 
			"testPath3").get();
		return new TrajectoryCommand(traj);
	}

	@Auto(name = "right neutral auto")
	public static Command rightAutoNeutral() {
		var tTrenchRight = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"TrenchRight");

		if (tTrenchRight.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var tSwipeRight = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"SwipeRight");

		if (tSwipeRight.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var tReturnTrenchRight = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"ReturnTrenchRight");

		if (tReturnTrenchRight.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var crossTrench = tTrenchRight.get();
		var swipe = tSwipeRight.get();
		var back = tReturnTrenchRight.get();
		return Commands.print(Timer.getFPGATimestamp() + ": Time start")
			.andThen(Intake.getInstance().calibrateZero())
				.alongWith(
					new TrajectoryCommand(crossTrench))
			.andThen(
				Intake.getInstance().intake())
			.andThen(
				new TrajectoryCommand(swipe))
			.andThen(
				Intake.getInstance().lower()
					.alongWith(
						new TrajectoryCommand(back)))
			.andThen(
				Drive.getInstance().headingLockToHub()
					.raceWith(
						Automation.shootAll()
							.andThen(
								Commands.waitSeconds(0.5))
							.andThen(
								Automation.indexAll())
							.andThen(
								Commands.waitSeconds(3))
							.andThen(
								Intake.getInstance().stow())
							.andThen(
								Commands.waitSeconds(3))))
			// .andThen(
			// 	new AutopilotCommand(
			// 		new APTarget(
			// 			new Pose2d(Constants.FieldConstants.Tower.rightUpright, Rotation2d.kCCW_90deg))
			// 			.withEntryAngle(Rotation2d.kZero)))
			.andThen(
				Commands.print(Timer.getFPGATimestamp() + ": Time end"),
				Commands.idle())
			.finallyDo(
				() -> {
					CommandScheduler.getInstance().schedule(
						Drive.getInstance().openLoopControl(),
						Intake.getInstance().lower(),
						Automation.stopShoot(),
						Automation.stopIndex());
				});
	}

	@Auto(name = "left neutral auto")
	public static Command leftAutoNeutral() {
		var tTrenchLeft = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"TrenchLeft");

		if (tTrenchLeft.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var tSwipeLeft = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"SwipeLeft");

		if (tSwipeLeft.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var tReturnTrenchLeft = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"ReturnTrenchLeft");

		if (tReturnTrenchLeft.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var crossTrench = tTrenchLeft.get();
		var swipe = tSwipeLeft.get();
		var back = tReturnTrenchLeft.get();
		return Commands.print(Timer.getFPGATimestamp() + ": Time start")
			.andThen(Intake.getInstance().calibrateZero())
			.alongWith(
				new TrajectoryCommand(crossTrench))
			.andThen(
				Intake.getInstance().intake())
			.andThen(
				new TrajectoryCommand(swipe))
			.andThen(
				Intake.getInstance().lower()
					.alongWith(
						new TrajectoryCommand(back)))
			.andThen(
				Drive.getInstance().headingLockToHub()
					.alongWith(
						Automation.shootAll()
							.andThen(
								Commands.waitSeconds(0.5))
							.andThen(
								Automation.indexAll()
									.andThen(
										Commands.waitSeconds(3))
									.andThen(
										Intake.getInstance().stow()))))
			.andThen(
				Commands.print(Timer.getFPGATimestamp() + ": Time end"),
				Commands.idle())
			.finallyDo(
				() -> {
					CommandScheduler.getInstance().schedule(
						Drive.getInstance().openLoopControl(),
						Intake.getInstance().lower(),
						Automation.stopShoot(),
						Automation.stopIndex());
				});
	}

	@Auto(name = "center auto")
	public static Command centerAuto() {
		var tDepotCenter = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"DepotCenter");

		if (tDepotCenter.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var tDepotShootCenter = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"DepotShootCenter");

		if (tDepotShootCenter.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var tStationCenter = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"StationCenter");

		if (tStationCenter.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var tStationShootCenter = TrajectoryLoader.loadAutoTrajectory(
			TrajectoryType.PATHPLANNER, 
			"StationShootCenter");

		if (tStationCenter.isEmpty()) {
			DriverStation.reportWarning(
				"Something happened", true);
			return Commands.none();
		}

		var depotCenter = tDepotCenter.get();
		var depotShootCenter = tDepotShootCenter.get();
		var stationCenter = tStationCenter.get();
		var stationShootCenter = tStationShootCenter.get();

		return Commands.print(Timer.getFPGATimestamp() + ": Time start")
			.andThen(Intake.getInstance().calibrateZero())
			.andThen(Intake.getInstance().intake())
			.alongWith(new TrajectoryCommand(depotCenter))
			.andThen(
				new TrajectoryCommand(depotShootCenter))
			.andThen(
				Drive.getInstance().headingLockToHub()
					.raceWith(
						Automation.shootAll()
							.andThen(
								Commands.waitSeconds(0.5))
							.andThen(
								Automation.indexAll())
							.andThen(
								Commands.waitSeconds(3))))
			.andThen(
				Intake.getInstance().lower(),
				Automation.stopShoot(),
				Automation.stopIndex())
			.andThen(
				new TrajectoryCommand(stationCenter))
			.andThen(
				Commands.waitSeconds(3))
			.andThen(
				new TrajectoryCommand(stationShootCenter))
			.andThen(
				Drive.getInstance().headingLockToHub()
					.alongWith(
						Automation.shootAll()
							.andThen(
								Commands.waitSeconds(0.5))
							.andThen(
								Automation.indexAll())
							.andThen(
								Commands.waitSeconds(3))
							.andThen(
								Intake.getInstance().stow())))
			.andThen(
				Commands.print(Timer.getFPGATimestamp() + ": Time end"),
				Commands.idle())
			.finallyDo(
				() -> {
					CommandScheduler.getInstance().schedule(
						Drive.getInstance().openLoopControl(),
						Intake.getInstance().lower(),
						Automation.stopShoot(),
						Automation.stopIndex());
				});
	}
}

