package frc.robot.auto;

import frc.robot.auto.AutoSelector.Auto;
import frc.robot.auto.Automation.Levels;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.trajectory.TrajectoryLoader;
import frc.robot.lib.trajectory.RedTrajectory.TrajectoryType;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public final class AutoRoutines {
	@Auto(name = "Pid Test")
	public static Command testPidToPose() {
		return new PIDToPoseCommand(
			new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(120)));
	}

	@Auto(name = "Trajectory Test")
	public static Command testTrajectoryAuto() {
		RedTrajectory traj = TrajectoryLoader.loadAutoTrajectory(TrajectoryType.PATHPLANNER, 
			"testPath2").get();
		return new TrajectoryCommand(traj);
	}

	@Auto(name = "Score Test")
	public static Command testAutoScore() {
		return Automation.score(true, Levels.L4);
	}
}

