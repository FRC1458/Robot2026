package frc.robot;

import static frc.robot.Robot.controller;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlsMapping {
	CommandXboxController controller1;
    
	public static void mapTeleopCommand() {
		
		
		
		Drive.getInstance().setDefaultCommand((Drive.getInstance().teleopCommand()));
		// run sysID functions
		Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER);
		
		controller.a().onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));
		controller.leftBumper().whileTrue(Drive.getInstance().autoAlign(true));
		controller.rightBumper().whileTrue(Drive.getInstance().autoAlign(false));
		controller.x().whileTrue(Drive.getInstance().autopilotAlign(true));
		controller.y().whileTrue(Drive.getInstance().autopilotAlign(false));
		controller.b().whileTrue(HangCommand()); // TODO: Implement hang from armaaan
		controller.leftTrigger().whileTrue(intakeCommand());
		controller.rightTrigger().whileTrue(shooterCommand());
	}

	public static Command intakeCommand() {
		return Commands.print("Intaking");
	}
	public static Command HangCommand() {
		return Commands.print("Hanging");
	}
	public static Command shooterCommand() {
		return Commands.print("Shooting");
		
	}
	
		//make a method that returns command, but it returns command.none, replace everything with new instance command with the command name.

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
