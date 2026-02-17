package frc.robot;

import static frc.robot.Robot.controller;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.lib.field.FieldUtil;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.vision.VisionDeviceManager;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControlsMapping {
    
	public static void mapTeleopCommand() {
		Drive.getInstance().setDefaultCommand((Drive.getInstance().openLoopControl()));
		Intake.getInstance().setDefaultCommand(Intake.getInstance().intake());
		
		controller.back().and(controller.a()).onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));
		controller.back().and(controller.b()).onTrue(VisionDeviceManager.getInstance().bootUp());

		controller.leftTrigger().debounce(0.1).whileTrue(Intake.getInstance().outtake());
		controller.b().whileTrue(Intake.getInstance().outtake());
		controller.x().onTrue(Climb.getInstance().hangCommand());

		controller.rightTrigger().debounce(0.1
		// ).onTrue(
		// 	Commands.parallel(
		// 		Shooter.getLeftInstance().shoot(50, -50),
		// 		Shooter.getRightInstance().shoot(50, -50))
		// ).onFalse(
		// 	Commands.parallel(
		// 		Shooter.getLeftInstance().stop(),
		// 		Shooter.getRightInstance().stop())
		).whileTrue(
			Commands.parallel(
				Indexer.getLeftInstance().activateIndexer(),
				Indexer.getRightInstance().activateIndexer())
		);

		controller.leftBumper().whileTrue(Drive.getInstance().headingLockToHub()
			.alongWith(Shooter.getLeftInstance().shoot(), Shooter.getRightInstance().shoot()));
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