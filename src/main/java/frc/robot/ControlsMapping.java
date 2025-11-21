package frc.robot;

import static frc.robot.Robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.coralshooter.CoralShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ControlsMapping {
	public static void mapTeleopCommand() {
		controller.back().onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));
		controller.leftBumper().whileTrue(Drive.getInstance().autoAlign(true));
		controller.rightBumper().whileTrue(Drive.getInstance().autoAlign(false));
		controller.a().onTrue(Elevator.getInstance().moveToScoringHeight(ElevatorConstants.Heights.BASE));
		controller.x().onTrue(Elevator.getInstance().moveToScoringHeight(ElevatorConstants.Heights.L2));
		controller.b().onTrue(Elevator.getInstance().moveToScoringHeight(ElevatorConstants.Heights.L3));
		controller.y().onTrue(Elevator.getInstance().moveToScoringHeight(ElevatorConstants.Heights.L4));
		controller.leftTrigger().onTrue(CoralShooter.getInstance().intake());
		controller.rightTrigger().onTrue(CoralShooter.getInstance().shoot());
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
