package frc.robot.auto;

import frc.robot.auto.AutoSelector.Auto;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.TrajectoryLoader;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.HeadingLockToHub2;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
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
		return 
			Intake.getInstance().calibrateZero()
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
				Drive.getInstance().headingLockToHub())
			.alongWith(
				Automation.shootAll()
					.andThen(
						Commands.waitSeconds(0.5))
					.andThen(
						Automation.indexAll()));
	}
}

