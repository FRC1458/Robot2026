package frc.robot;

import static frc.robot.Robot.controller;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControlsMapping {
	public static void mapTeleopCommand() {
		Drive.getInstance().setDefaultCommand((Drive.getInstance().openLoopControl()));
		// run sysID functions
		Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER);
		
		controller.a().onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));

		controller.b().whileTrue(Drive.getInstance().headingLockToPose(DriveConstants.FieldPoses.HUB.pose));
		controller.x().onTrue(Drive.getInstance().pathFindToThisRandomPlaceIdk());
		// controller.leftBumper().whileTrue(Drive.getInstance().autoAlign(true));
		// controller.rightBumper().whileTrue(Drive.getInstance().autoAlign(false));
		// controller.x().whileTrue(Drive.getInstance().autopilotAlign(true));
		// controller.y().whileTrue(Drive.getInstance().autopilotAlign(false));
	}

	public static void mapSysId() {
		// set up sysID routine type
		controller.a().onTrue(Commands.runOnce(
			() -> Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.TRANSLATION)));
		controller.b().onTrue(Commands.runOnce(
			() -> Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.ROTATION)));
		controller.back().onTrue(Commands.runOnce(
			() -> Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER)));
		// map the sysid routine movement directions
		controller.leftBumper().and(controller.x()).whileTrue(
			Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kForward)
				.finallyDo((
					boolean interrupted) -> {
						if (interrupted) {
							Drive.getInstance().setSwerveRequest(new SwerveRequest.Idle());
						}
					}));
		controller.leftBumper().and(controller.x()).whileTrue(
			Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kReverse)
				.finallyDo((
					boolean interrupted) -> {
						if (interrupted) {
							Drive.getInstance().setSwerveRequest(new SwerveRequest.Idle());
						}
					}));
		controller.rightBumper().and(controller.x()).whileTrue(
			Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kForward)
				.finallyDo((
					boolean interrupted) -> {
						if (interrupted) {
							Drive.getInstance().setSwerveRequest(new SwerveRequest.Idle());
						}
					}));
		controller.rightBumper().and(controller.y()).whileTrue(
			Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kReverse)
				.finallyDo((
					boolean interrupted) -> {
						if (interrupted) {
							Drive.getInstance().setSwerveRequest(new SwerveRequest.Idle());
						}
					}));
	}
}
