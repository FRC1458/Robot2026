package frc.robot;

import static frc.robot.Robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;
//import frc.robot.subsystems.drive.commands.TeleopCommand;
import frc.robot.subsystems.drive.ctre.CtreDrive.SysIdRoutineType;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterCommands.SpinFast;
import frc.robot.subsystems.Shooter.ShooterCommands.Spin;
import frc.robot.subsystems.Shooter.ShooterCommands.Reverse;

public class ControlsMapping {

	private static final ShooterSubsystem shooter = new ShooterSubsystem();

	public static void mapTeleopCommand() {

		Drive.getInstance().setDefaultCommand((Drive.getInstance().teleopCommand()));
		// run sysID functions
		Drive.getInstance().getCtreDrive().setSysIdRoutine(SysIdRoutineType.STEER);
		
		controller.a().onTrue(Drive.getInstance().resetPoseCommand(new Pose2d()));

		controller.leftBumper().whileTrue(new Reverse(shooter));
		controller.rightBumper().whileTrue(new Spin(shooter));
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
