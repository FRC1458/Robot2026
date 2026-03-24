package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
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
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.vision.VisionDeviceManager;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;


public class ControlsMapping {

	public static void mapTeleopCommand() {
		Drive.getInstance().setDefaultCommand((Drive.getInstance().openLoopControl()));
		// run sysID functions
		Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER);
		
		controller.a().onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));
		controller.leftBumper().whileTrue(Drive.getInstance().autoAlign(true));
		controller.rightBumper().whileTrue(Drive.getInstance().autoAlign(false));
		controller.x().whileTrue(Drive.getInstance().autopilotAlign(true));
		controller.y().whileTrue(Drive.getInstance().autopilotAlign(false));
		new Trigger(() -> controller.getHID().getPOV() != -1).whileTrue(Drive.getInstance().nudgeCommand());

		controller.b().whileTrue(Drive.getInstance().headingLockToPose(DriveConstants.FieldPoses.TAG.pose));
		controller.x().onTrue(Drive.getInstance().pathFindToThisRandomPlaceIdk());
		// controller.leftBumper().whileTrue(Drive.getInstance().autoAlign(true));
		// controller.rightBumper().whileTrue(Drive.getInstance().autoAlign(false));
		// controller.x().whileTrue(Drive.getInstance().autopilotAlign(true));
		// controller.y().whileTrue(Drive.getInstance().autopilotAlign(false));
		controller.y().onTrue(VisionDeviceManager.getInstance().bootUp());
		// Intake.getInstance().setDefaultCommand(Intake.getInstance().intake());

		controller.back().onTrue(Drive.getInstance().resetPoseCommand(new
			Pose2d()));
		controller.y().onTrue(VisionDeviceManager.getInstance().bootUp());

		// controller.leftTrigger().debounce(0.1).whileTrue(Intake.getInstance().outtake());
		// controller.b().whileTrue(Intake.getInstance().outtake());
		// controller.x().onTrue(Climb.getInstance().hangCommand());

		// controller.rightTrigger().debounce(0.1
		// // ).onTrue(
		// // Commands.parallel(
		// // Shooter.getLeftInstance().shoot(50, -50),
		// // Shooter.getRightInstance().shoot(50, -50))
		// // ).onFalse(
		// // Commands.parallel(
		// // Shooter.getLeftInstance().stop(),
		// // Shooter.getRightInstance().stop())
		// ).whileTrue(
		// Commands.parallel(
		// Indexer.getLeftInstance().activateIndexer(),
		// Indexer.getRightInstance().activateIndexer())
		// ).whileTrue(
		// Commands.repeatingSequence(
		// Commands.runOnce(() -> Robot.fuelSim.launchFuel(
		// MetersPerSecond.of(
		// Shooter.getLeftInstance().getTopSpeed() * Constants.TAU * 0.0508),
		// Degrees.of(75),
		// Degrees.of(0),
		// Inches.of(19)
		// )).andThen(Commands.waitSeconds(0.1))
		// )
		// );
		controller.a().whileTrue(
			Commands.parallel(
				Shooter.getRightInstance().shoot(60, 60),
				Shooter.getLeftInstance().shoot(60, 60)
			)
		).onFalse(
			Commands.parallel(
				Shooter.getRightInstance().stop(),
				Shooter.getLeftInstance().stop()
			)
		);

		controller.x().whileTrue(
			Commands.parallel(
				Indexer.getRightInstance().activateIndexer(),
				Indexer.getLeftInstance().activateIndexer(),
				Roller.getInstance().roll()
			)
		).onFalse(
			Commands.parallel(
				Indexer.getRightInstance().deactivateIndexer(),
				Indexer.getLeftInstance().deactivateIndexer(),
				Roller.getInstance().stop()
			)
		);


		controller.b().whileTrue(
				Intake.getInstance().intake()).onFalse(Intake.getInstance().stow());
		// controller.y().onTrue(Intake.getInstance().)
		controller.leftBumper().whileTrue(Drive.getInstance().headingLockToHub());
		// Shooter.getRightInstance().shoot()));
	}

	public static void mapSysId() {
		// set up sysID routine type
		controller.a().onTrue(new InstantCommand(()->Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.TRANSLATION)));
		controller.b().onTrue(new InstantCommand(()->Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.ROTATION)));
		controller.back().onTrue(new InstantCommand(()->Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER)));
		// map the sysid routine movement directions
		controller.leftBumper().and(controller.x())
			.whileTrue(
				new ProxyCommand(
					()->Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kForward)
						.finallyDo(interrupted->Drive.getInstance().getCtreDrive().setControl(new SwerveRequest.Idle()))
				)
			);
		controller.leftBumper().and(controller.y())
			.whileTrue(
				new ProxyCommand(
					()->Drive.getInstance().getCtreDrive().sysIdDynamic(Direction.kReverse)
						.finallyDo(interrupted->Drive.getInstance().getCtreDrive().setControl(new SwerveRequest.Idle()))
				)
			);
		controller.rightBumper().and(controller.x())
				.whileTrue(
					new ProxyCommand(
						()->Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kForward)
							.finallyDo(interrupted->Drive.getInstance().getCtreDrive().setControl(new SwerveRequest.Idle()))
					)
				);
		controller.rightBumper().and(controller.y())
				.whileTrue(
					new ProxyCommand(
						()->Drive.getInstance().getCtreDrive().sysIdQuasistatic(Direction.kReverse)
							.finallyDo(interrupted->Drive.getInstance().getCtreDrive().setControl(new SwerveRequest.Idle()))
					)
				);
	}

}