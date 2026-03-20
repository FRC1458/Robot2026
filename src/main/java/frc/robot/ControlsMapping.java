package frc.robot;

import static frc.robot.Robot.controller;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionDeviceManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class ControlsMapping {

	public static LoggedNetworkNumber speed = new LoggedNetworkNumber("speed", 30);
	public static LoggedNetworkNumber bottom = new LoggedNetworkNumber("bottom", 30);

	public static void mapTeleopCommand() {

		Drive.getInstance().setDefaultCommand((Drive.getInstance().openLoopControl()));

		controller.back().and(controller.a()).onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));
		controller.back().and(controller.b()).onTrue(VisionDeviceManager.getInstance().bootUp());
		controller.back().and(controller.y()).onTrue(
				Commands.runOnce(() -> Drive.getInstance().getCtreDrive().getPigeon2().reset()));

		// controller.x().whileTrue(Intake.getInstance().outtake());

	
		controller.rightBumper()
			.whileTrue(shootAll())
			.onFalse(stopShoot());
		controller.leftBumper()
			.whileTrue(indexAll())
			.onFalse(stopIndex());

		controller.leftTrigger()
			.whileTrue(Intake.getInstance().intake())
			.onFalse(Intake.getInstance().stopWheel());

		controller.y().and(controller.back().negate())
			.whileTrue(Intake.getInstance().stow())
			.onFalse(Intake.getInstance().lower());

		controller.rightTrigger().whileTrue(Drive.getInstance().headingLockToHub());
		controller.povDown().onTrue(Intake.getInstance().calibrateZero());
		controller.b().and(controller.back().negate()).whileTrue(backIndex()).onFalse(stopIndex());

		controller.y().and(controller.back().negate()).whileTrue(Shooter.getRightInstance().sysId().dynamic(SysIdRoutine.Direction.kForward));
		controller.x().and(controller.back().negate()).whileTrue(Shooter.getRightInstance().sysId().dynamic(SysIdRoutine.Direction.kReverse));
		controller.b().and(controller.back().negate()).whileTrue(Shooter.getRightInstance().sysId().quasistatic(SysIdRoutine.Direction.kForward));
		controller.a().and(controller.back().negate()).whileTrue(Shooter.getRightInstance().sysId().quasistatic(SysIdRoutine.Direction.kReverse));
		// controller.a().whileTrue(
		// 	Intake.getInstance().agitate()
		// ).onFalse(Intake.getInstance().lower());

		// controller.rightBumper().whileTrue(
		// 	Commands.parallel(
		// 		Shooter.getRightInstance().shoot(
		// 			() -> SmartDashboard.getNumber("rightv", 30.0) - 15,
		// 			() -> SmartDashboard.getNumber("rightv", 30.0) + 15
		// 		),
		// 		Shooter.getLeftInstance().shoot(
		// 			() -> SmartDashboard.getNumber("leftv", 30.0) - 15,
		// 			() -> SmartDashboard.getNumber("leftv", 30.0) + 15
		// 		)
		// 	)
		// 	).onFalse(
		// 		stopShoot()
		// 	);

	}

	public static Command shootAll() {
		return Commands.parallel(
			Shooter.getRightInstance().shoot(),
			Shooter.getLeftInstance().shoot());
	}
	public static Command shootAllTable() {
		return Commands.parallel(
			Shooter.getRightInstance().shoot(speed::getAsDouble, speed::getAsDouble),
			Shooter.getLeftInstance().shoot(speed::getAsDouble, speed::getAsDouble));
	}

	public static Command stopShoot() {
		return Commands.parallel(
			Shooter.getRightInstance().stop(),
			Shooter.getLeftInstance().stop());
	}

	public static Command indexAll() {
		return Commands.parallel(
			Indexer.getRightInstance().activateIndexer(),
			Indexer.getLeftInstance().activateIndexer(),
			Roller.getInstance().roll());
	}

	public static Command stopIndex() {
		return Commands.parallel(
			Indexer.getRightInstance().deactivateIndexer(),
			Indexer.getLeftInstance().deactivateIndexer(),
			Roller.getInstance().stop());
	}

	public static Command backIndex() {
		return Commands.parallel(
				Indexer.getRightInstance().back(),
				Indexer.getLeftInstance().back(),
				Roller.getInstance().antiRoll());
	}

	@SuppressWarnings("removal")
	public static void mapSysId() {
		// set up sysID routine type
		controller.a().onTrue(new InstantCommand(
				() -> Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.TRANSLATION)));
		controller.b().onTrue(new InstantCommand(
				() -> Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.ROTATION)));
		controller.back().onTrue(
				new InstantCommand(() -> Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER)));
		// map the sysid routine movement directions
		controller.leftBumper().and(controller.x())
				.whileTrue(
						new ProxyCommand(
								() -> Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kForward)
										.finallyDo(interrupted -> Drive.getInstance().getCtreDrive()
												.setControl(new SwerveRequest.Idle()))));
		controller.leftBumper().and(controller.y())
				.whileTrue(
						new ProxyCommand(
								() -> Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kReverse)
										.finallyDo(interrupted -> Drive.getInstance().getCtreDrive()
												.setControl(new SwerveRequest.Idle()))));
		controller.rightBumper().and(controller.x())
				.whileTrue(
						new ProxyCommand(
								() -> Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kForward)
										.finallyDo(interrupted -> Drive.getInstance().getCtreDrive()
												.setControl(new SwerveRequest.Idle()))));
		controller.rightBumper().and(controller.y())
				.whileTrue(
						new ProxyCommand(
								() -> Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kReverse)
										.finallyDo(interrupted -> Drive.getInstance().getCtreDrive()
												.setControl(new SwerveRequest.Idle()))));
	}
}