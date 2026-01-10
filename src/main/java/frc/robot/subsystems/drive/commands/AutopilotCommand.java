package frc.robot.subsystems.drive.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class AutopilotCommand extends Command {
	private final APTarget target;
	private final Drive drive;
	private static final APConstraints constraints = new APConstraints()
		.withVelocity(MAX_SPEED)
		.withAcceleration(MAX_ACCEL)
		.withJerk(3.0);

	private static final APProfile profile = new APProfile(constraints)
		.withErrorXY(Units.Meters.of(DriveConstants.EPSILON_TRANSLATION))
		.withErrorTheta(Units.Radians.of(DriveConstants.EPSILON_ROTATION))
		.withBeelineRadius(Units.Centimeters.of(8));

	public static final Autopilot autoPilot = new Autopilot(profile);

	private final SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
		.withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
		.withDriveRequestType(DriveRequestType.Velocity)
		.withHeadingPID(
			ROTATION_CONSTANTS.kP,
			ROTATION_CONSTANTS.kI,
			ROTATION_CONSTANTS.kD);

	public AutopilotCommand(APTarget target) {
		this(target, Drive.getInstance());
	}

	public AutopilotCommand(APTarget target, Drive drive) {
		this.target = target;
		this.drive = drive;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		drive.setSwerveRequest(request);
	}

	@Override
	public void execute() {
		ChassisSpeeds robotRelativeSpeeds = drive.getState().Speeds;
		Pose2d pose = drive.getPose();

		APResult out = autoPilot.calculate(pose, robotRelativeSpeeds, target);

		request
			.withVelocityX(out.vx())
			.withVelocityY(out.vy())
			.withTargetDirection(out.targetAngle());
	}

	@Override
	public boolean isFinished() {
		return autoPilot.atTarget(drive.getPose(), target);
	}

	@Override
	public void end(boolean interrupted) {
		drive.setSwerveRequest(new SwerveRequest.RobotCentric());
	}
}
