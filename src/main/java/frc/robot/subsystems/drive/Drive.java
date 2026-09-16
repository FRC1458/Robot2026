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
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.lib.trajectory.RedTrajectory.State.ChassisAccels;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.commands.AutopilotCommand;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
import frc.robot.subsystems.drive.ctre.CompCtreDriveConstants;
import frc.robot.subsystems.drive.ctre.CtreDrive;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class Drive extends SubsystemBase {
	private SwerveDriveState lastReadState;
	public static SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric();
	public SwerveRequest driveRequest = teleopRequest;
	private ChassisSpeeds prevSpeeds = new ChassisSpeeds();

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
		prevSpeeds = getFieldSpeeds();
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
		return lastReadState;
	}

	/**
	 * @return the last read pose
	 */
	public Pose2d getPose() {
		return lastReadState.Pose;
	}

	/**
	 * @return the last read pose
	 */
	public Rotation2d getRotation() {
		return lastReadState.Pose.getRotation();
	}

	/**
	 * @return the chassis speeds, field relative
	 */
	public ChassisSpeeds getRobotSpeeds() {
		return lastReadState.Speeds;
	}

	/**
	 * @return the chassis speeds, field relative
	 */
	public ChassisSpeeds getFieldSpeeds() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(
				lastReadState.Speeds, lastReadState.Pose.getRotation());
	}

	public ChassisAccels getAccel() {
		return ChassisAccels.estimate(prevSpeeds, getFieldSpeeds(), 0.02);
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

	public ChassisSpeeds getChassisSpeedsFromController() {
		double xDesiredRaw = -Robot.controller.getLeftY();
		double yDesiredRaw = -Robot.controller.getLeftX();
		double rotDesiredRaw = -Robot.controller.getRightX();

		double[] xy =
				Util.applyRadialDeadband(
						xDesiredRaw, yDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);
		double xFancy = Math.pow(xy[0], 3);
		double yFancy = Math.pow(xy[1], 3);
		double rotFancy =
				Util.applyJoystickDeadband(
						rotDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);

		return new ChassisSpeeds(xFancy, yFancy, Math.pow(rotFancy, 3));
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
									ChassisSpeeds fromController = getChassisSpeedsFromController();

									teleopRequest
											.withVelocityX(fromController.vxMetersPerSecond * MAX_SPEED)
											.withVelocityY(fromController.vyMetersPerSecond * MAX_SPEED)
											.withRotationalRate(fromController.omegaRadiansPerSecond * MAX_ROTATION_SPEED);
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
									getRotation().getRadians(), getRobotSpeeds().omegaRadiansPerSecond);
						})
				.andThen(
						run(() -> {
									ChassisSpeeds fromController = getChassisSpeedsFromController();

									var delta = pose.minus(getPose().getTranslation());
									var targetDirection = delta.getAngle();

									var normSq = delta.getNorm() * delta.getNorm();
									var fieldSpeeds = getFieldSpeeds();
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
															getRotation().getRadians(), getRobotSpeeds().omegaRadiansPerSecond)
													.getOutput();

									SmartDashboard.putNumber(
											"error tracking",
											MathUtil.inputModulus(
													getRotation().minus(targetDirection).getDegrees(), -180, 180));

									request
											.withVelocityX(fromController.vxMetersPerSecond * MAX_SPEED)
											.withVelocityY(fromController.vyMetersPerSecond * MAX_SPEED)
											.withRotationalRate(rotation);
								})
								.handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric())))
				.withName("Heading Lock");
	}

	/** Locks the robot onto a pose. Utilizes feedforwards derived from the current chassis speeds */
	public Command headingLockToPose(Supplier<Translation2d> poseSupplier) {
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
									getRotation().getRadians(), getRobotSpeeds().omegaRadiansPerSecond);
						})
				.andThen(
						run(() -> {
									ChassisSpeeds fromController = getChassisSpeedsFromController();

									var delta = poseSupplier.get().minus(getPose().getTranslation());
									var targetDirection = delta.getAngle();

									var normSq = delta.getNorm() * delta.getNorm();
									var fieldSpeeds = getFieldSpeeds();
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
															getRotation().getRadians(), getRobotSpeeds().omegaRadiansPerSecond)
													.getOutput();

									SmartDashboard.putNumber(
											"error tracking",
											MathUtil.inputModulus(
													getRotation().minus(targetDirection).getDegrees(), -180, 180));

									request
											.withVelocityX(fromController.vxMetersPerSecond * MAX_SPEED)
											.withVelocityY(fromController.vyMetersPerSecond * MAX_SPEED)
											.withRotationalRate(rotation);
								})
								.handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric())))
				.withName("Heading Lock");
	}

	public boolean isPointedTowardsPos(Translation2d pos, double eps) {
		var direction = pos.minus(getPose().getTranslation()).getAngle();
		var current = getRotation();
		return Math.abs(MathUtil.inputModulus(direction.getDegrees() - current.getDegrees(), -180, 180))
				< eps;
	}

	/**
	 * Locks the robot onto a pose, with TOF Adjustment Utilizes feedforwards derived from the current
	 * chassis speeds
	 */
	public Command headingLockToHub() {
		return headingLockToPose(() -> Constants.FieldConstants.hubLocation);
	}

	public Command waitUntilAligned() {
		return Commands.waitUntil(() -> isPointedTowardsPos(Constants.FieldConstants.hubLocation, 10));
	}

	public Command directionHeadingLock(Supplier<Rotation2d> rotation) {
		SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

		ProfiledPIDVController thetaController =
				new ProfiledPIDVController(
						new ProfiledPIDVConstants(
								new PIDVConstants(10.0, 0.0, 1),
								new TrapezoidProfile.Constraints(Math.PI * 16, Math.PI * 5)));
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		AtomicReference<Rotation2d> pose = new AtomicReference<Rotation2d>(Rotation2d.kZero);

		return runOnce(
						() -> {
							request.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
							setSwerveRequest(request);

							thetaController.setInitialSetpoint(
									getRotation().getRadians(), getRobotSpeeds().omegaRadiansPerSecond);
							pose.set(rotation.get());
						})
				.andThen(
						run(() -> {
									ChassisSpeeds fromController = getChassisSpeedsFromController();

									var targetDirection = pose.get();

									var r =
											thetaController
													.setTarget(targetDirection.getRadians())
													.setMeasurement(
															getRotation().getRadians(), getRobotSpeeds().omegaRadiansPerSecond)
													.getOutput();

									request
											.withVelocityX(fromController.vxMetersPerSecond * MAX_SPEED)
											.withVelocityY(fromController.vyMetersPerSecond * MAX_SPEED)
											.withRotationalRate(r);
								})
								.handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric())))
				.withName("Heading Lock");
	}

	public Command passAlign() {
		return directionHeadingLock(() -> Constants.isBlue ? Rotation2d.k180deg : Rotation2d.kZero);
	}

	public Command waitUntilAlignedPass() {
		return Commands.waitUntil(() -> isPointedPass(30));
	}

	public boolean isPointedPass(double eps) {
		return Constants.isBlue
				? Math.abs(MathUtil.inputModulus(getRotation().getDegrees() - 180, -180, 180)) < eps
				: Math.abs(MathUtil.inputModulus(getRotation().getDegrees(), -180, 180)) < eps;
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
		ChassisSpeeds speeds = getRobotSpeeds();
		return isPitchStable()
				&& isRollStable()
				&& Units.MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
						.lte(MAX_SPEED_SCORING_TRANSLATION)
				&& Units.RadiansPerSecond.of(speeds.omegaRadiansPerSecond).lte(MAX_ROTATION_SPEED_SCORING);
	}
}
