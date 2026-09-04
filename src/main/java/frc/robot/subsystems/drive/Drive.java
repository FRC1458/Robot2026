package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.therekrab.autopilot.APTarget;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.control.ControlConstants.PIDVConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDVConstants;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.field.FieldLayout;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.commands.AutopilotCommand;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
import frc.robot.subsystems.drive.ctre.CompCtreDriveConstants;
import frc.robot.subsystems.drive.ctre.CtreDrive;

public class Drive extends SubsystemBase {
	private SwerveDriveState lastReadState;
	public static SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric();
	public SwerveRequest driveRequest = teleopRequest;

	private final CtreDrive drivetrain;

	public Drive() {
		super("Drive");
		drivetrain = CompCtreDriveConstants.createDrivetrain();
		teleopRequest = new SwerveRequest.FieldCentric();
		driveRequest = teleopRequest;
		lastReadState = drivetrain.getState();
		drivetrain.setDefaultCommand(
				drivetrain.applyRequest(
						() -> {
							return driveRequest;
						}));

		drivetrain.getOdometryThread().setThreadPriority(31);
		TelemetryManager.getInstance()
				.addStructPublisher("Mechanisms/Drive", Pose3d.struct, () -> new Pose3d(getPose()));

		setDefaultCommand(openLoopControl());
	}

	/**
	 * @return the ctre generated drivetrain
	 */
	public CtreDrive getCtreDrive() {
		return drivetrain;
	}

	@Override
	public void periodic() {
		lastReadState = drivetrain.getState();
		outputTelemetry();
	}

	public void outputTelemetry() {
		FieldLayout.field.setRobotPose(getPose());
		var state = lastReadState;
		DogLog.log(getName() + "/Pose", state.Pose);
		DogLog.log(getName() + "/RobotSpeeds", state.Speeds);
		DogLog.log(getName() + "/FieldSpeeds", getFieldSpeeds());
		DogLog.log(getName() + "/ModuleStates", state.ModuleStates);
		DogLog.log(getName() + "/ModulePositions", state.ModulePositions);
		DogLog.log(getName() + "/ModuleTargets", state.ModuleTargets);
	}

	/**
	 * @return the current state
	 */
	public SwerveDriveState getState() {
		return drivetrain.getState();
	}

	/**
	 * @return the last read pose
	 */
	public Pose2d getPose() {
		return lastReadState.Pose;
	}

	/**
	 * @return the chassis speeds, field relative
	 */
	public ChassisSpeeds getFieldSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(
				lastReadState.Speeds, lastReadState.Pose.getRotation());
	}

	/**
	 * Switches the swerve request
	 *
	 * <p>Please do not the new swerve request every 20 ms
	 */
	public void setSwerveRequest(SwerveRequest request) {
		driveRequest = request;
	}

	/**
	 * @return the current swerve request
	 */
	public SwerveRequest getSwerveRequest() {
		return driveRequest;
	}

	/** Open loop during teleop */
	public Command openLoopControl() {
		return runOnce(
						() -> {
							teleopRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
							setSwerveRequest(teleopRequest);
						})
				.andThen(
						run(() -> {
									double xDesiredRaw = -Robot.controller.getLeftY();
									double yDesiredRaw = -Robot.controller.getLeftX();
									double rotDesiredRaw = -Robot.controller.getRightX();

									double[] xy =
											Util.applyRadialDeadband(
													xDesiredRaw, yDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);
									double xFancy = xy[0];
									double yFancy = xy[1];
									double rotFancy =
											Util.applyJoystickDeadband(
													rotDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);

									teleopRequest
											.withVelocityX(xFancy * MAX_SPEED)
											.withVelocityY(yFancy * MAX_SPEED)
											.withRotationalRate(rotFancy * MAX_ROTATION_SPEED);
								})
								.handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric())))
				.withName("Teleop");
	}

	/** Locks the robot onto a pose. Utilizes feedforwards derived from the current chassis speeds */
	public Command headingLockToPose(Translation2d pose) {
		SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

		ProfiledPIDVController thetaController =
				new ProfiledPIDVController(
						new ProfiledPIDVConstants(
								new PIDVConstants(10.0, 0.0, 1),
								new TrapezoidProfile.Constraints(Math.PI * 16, Math.PI * 5)));
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		return runOnce(
						() -> {
							request.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
							setSwerveRequest(request);

							thetaController.setInitialSetpoint(
									getPose().getRotation().getRadians(), getState().Speeds.omegaRadiansPerSecond);
						})
				.andThen(
						run(() -> {
									double xDesiredRaw = -Robot.controller.getLeftY();
									double yDesiredRaw = -Robot.controller.getLeftX();

									double[] xy =
											Util.applyRadialDeadband(
													xDesiredRaw, yDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);
									double xFancy = xy[0];
									double yFancy = xy[1];

									var state = getState();
									var delta = pose.minus(getPose().getTranslation());
									var targetDirection = delta.getAngle();

									var normSq = delta.getNorm() * delta.getNorm();
									var fieldSpeeds =
											ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, getPose().getRotation());
									var rotationalRate =
											normSq > 1e-4
													? (-delta.getX() * fieldSpeeds.vyMetersPerSecond
																	+ delta.getY() * fieldSpeeds.vxMetersPerSecond)
															/ (normSq)
													: 0.0;

									var rotation =
											thetaController
													.setTarget(targetDirection.getRadians(), rotationalRate)
													.setMeasurement(
															state.Pose.getRotation().getRadians(),
															state.Speeds.omegaRadiansPerSecond)
													.getOutput();

									SmartDashboard.putNumber(
											"error tracking",
											MathUtil.inputModulus(
													state.Pose.getRotation().minus(targetDirection).getDegrees(), -180, 180));

									request
											.withVelocityX(xFancy * MAX_SPEED)
											.withVelocityY(yFancy * MAX_SPEED)
											.withRotationalRate(rotation);
								})
								.handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric())))
				.withName("Heading Lock");
	}

	public boolean isPointedTowardsPos(Translation2d pos, double eps) {
		var direction = pos.minus(getPose().getTranslation()).getAngle();
		var current = getPose().getRotation();
		return Math.abs(MathUtil.inputModulus(direction.getDegrees() - current.getDegrees(), -180, 180))
				< eps;
	}

	/**
	 * Locks the robot onto a pose, with TOF Adjustment Utilizes feedforwards derived from the current
	 * chassis speeds
	 */
	public Command headingLockToHub() {
		return headingLockToPose(Constants.FieldConstants.hubLocation);
	}

	public Command waitUntilAligned() {
		return Commands.waitUntil(
				() -> isPointedTowardsPos(Constants.FieldConstants.hubLocation, 10));
	}

	public Command autoAlign(Pose2d targetPose) {
		return new PIDToPoseCommand(this, targetPose);
	}

	public TrajectoryCommand trajectory(RedTrajectory traj) {
		return new TrajectoryCommand(this, traj);
	}

	public Command dance() {
		double meanAngle =
				Constants.FieldConstants.hubLocation
						.minus(getPose().getTranslation())
						.getAngle()
						.getRadians();

		double range = 10;

		SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

		return headingLockToHub()
				.andThen(runOnce(() -> setSwerveRequest(request)))
				.andThen(
						Commands.sequence(
										runOnce(() -> request.withRotationalRate(meanAngle + range / 2)),
										Commands.waitSeconds(0.5),
										runOnce(() -> request.withRotationalRate(meanAngle - range / 2)),
										Commands.waitSeconds(0.5))
								.repeatedly())
				.handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric()))
				.withName("Dance");
	}

	public double getDistanceToHub() {
		return Constants.FieldConstants.hubLocation.getDistance(getPose().getTranslation());
	}

	/**
	 * Auto aligns to the nearest reef face
	 *
	 * @param left chooses the left or right face
	 */
	public Command autopilotAlign() {
		return defer(
						() -> {
							APTarget pose = FieldLayout.getNearestTarget(getPose());
							return new AutopilotCommand(pose, this);
						})
				.withName("Autopilot Align");
	}

	/** Adds a vision update */
	public void addVisionUpdate(Pose2d pose, double timestamp) {
		getCtreDrive().addVisionMeasurement(pose, timestamp);
	}

	/** Adds a vision update with standard deviations */
	public void addVisionUpdate(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
		getCtreDrive().addVisionMeasurement(pose, timestamp, stdDevs);
	}

	/** Resets pose estimator to a pose */
	public void resetPose(Pose2d pose) {
		getCtreDrive().resetPose(pose);
	}

	/** A command that resets the pose */
	public Command resetPoseCommand(Pose2d pose) {
		return Commands.runOnce(() -> resetPose(pose));
	}

	/** Whether the pitch is stable */
	public boolean isPitchStable() {
		return drivetrain
								.getPigeon2()
								.getAngularVelocityYDevice()
								.getValue()
								.abs(Units.DegreesPerSecond)
						< MAX_VELOCITY_STABLE
				&& drivetrain.getPigeon2().getPitch().getValue().abs(BaseUnits.AngleUnit)
						< MAX_PITCH_STABLE;
	}

	/** Whether the roll is stable */
	public boolean isRollStable() {
		return drivetrain
								.getPigeon2()
								.getAngularVelocityXDevice()
								.getValue()
								.abs(Units.DegreesPerSecond)
						< MAX_VELOCITY_STABLE
				&& drivetrain.getPigeon2().getRoll().getValue().abs(BaseUnits.AngleUnit) < MAX_PITCH_STABLE;
	}

	/** Whether the robot is stable */
	public boolean isStable() {
		ChassisSpeeds speeds = getState().Speeds;
		return isPitchStable()
				&& isRollStable()
				&& Units.MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
						.lte(MAX_SPEED_SCORING_TRANSLATION)
				&& Units.RadiansPerSecond.of(speeds.omegaRadiansPerSecond).lte(MAX_ROTATION_SPEED_SCORING);
	}
}
