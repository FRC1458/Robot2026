package frc.robot;

import static frc.robot.Robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;

public class ControlsMapping {
	public static void mapTeleopCommand() {
		controller.back().onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));
		controller.leftBumper().whileTrue(Drive.getInstance().autoAlign(true));
		controller.rightBumper().whileTrue(Drive.getInstance().autoAlign(false));
	}

	public static void mapSysId() {
		// run sysID functions
		Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER);
		controller.a().whileTrue(
			Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kForward));
		controller.b().whileTrue(
			Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kReverse));
		controller.x().whileTrue(
			Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kForward));
		controller.y().whileTrue(
			Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kReverse));
	}
}
