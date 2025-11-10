package frc.robot;

import static frc.robot.Robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
//import frc.robot.subsystems.drive.commands.TeleopCommand;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ControlsMapping {
	public static void mapTeleopCommand() {
		Drive.getInstance().setDefaultCommand(Drive.getInstance().teleopCommand());
		// run sysID functions
		Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER);
		
		controller.a().onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));
		controller.leftBumper().whileTrue(Drive.getInstance().autoAlign(true));
		controller.rightBumper().whileTrue(Drive.getInstance().autoAlign(false));
		controller.b().whileTrue(new PIDToPoseCommand(
			new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(120.0))));
		controller.x().onTrue(Elevator.getInstance().moveToScoringHeight(ElevatorConstants.Heights.BASE));
		controller.y().onTrue(Elevator.getInstance().moveToScoringHeight(ElevatorConstants.Heights.L2));
	}

	public static void mapSysId() {
		controller.a().onTrue(
			Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kForward));
		controller.b().onTrue(
			Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kReverse));
		controller.x().onTrue(
			Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kForward));
		controller.y().onTrue(
			Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kReverse));
	}
}
