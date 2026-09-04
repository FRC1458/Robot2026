package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.lib.field.FieldLayout;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants.VisionDeviceConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class VisionDevice {
	private final VisionDeviceConstants constants;

	public Field2d robotField;
	public PhotonCamera camera;
	public Pose2d botPose;

	private PhotonCameraSim sim;
	private boolean hasTarget;
	private boolean isConnected;
	private double latestTimestamp = 0.0;
	private Drive drive;

	public VisionDevice(VisionDeviceConstants constants, Drive drive) {
		this.constants = constants;
		this.drive = drive;
		this.robotField = new Field2d();
		this.camera = new PhotonCamera(constants.tableName);
		this.hasTarget = false;

		SmartDashboard.putData("VisionDevice/" + constants.tableName, robotField);

		if (Robot.isSimulation()) {
			SimCameraProperties cameraProp = new SimCameraProperties();
			cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
			cameraProp.setCalibError(0.25, 0.08);
			cameraProp.setFPS(20);
			cameraProp.setAvgLatencyMs(35);
			cameraProp.setLatencyStdDevMs(5);

			sim = new PhotonCameraSim(camera, cameraProp);
		}

		TelemetryManager.getInstance()
				.addStructPublisher(constants.name() + "Pose", Pose2d.struct, () -> botPose);
	}

	private void processFrames() {
		var result = camera.getLatestResult();
		hasTarget = result.hasTargets();

		if (!hasTarget) {
			robotField.setRobotPose(Pose2d.kZero);
			return;
		}

		Pose2d visionPose;
		double xStdev, yStdev, thetaStdev;
		double timestamp = result.getTimestampSeconds();
		this.latestTimestamp = timestamp; // Track for connection logic

		// Get current state for fallbacks
		Rotation2d currentGyroRotation = drive.getPose().getRotation();
		var multiTagResult = result.getMultiTagResult();

		if (multiTagResult.isPresent()) {
			// --- MULTI-TAG CASE ---
			var multiTag = multiTagResult.get();
			Pose3d cameraPose =
					new Pose3d(
							multiTag.estimatedPose.best.getTranslation(),
							multiTag.estimatedPose.best.getRotation());

			// Adjust for the Camera-to-Robot offset to get the ROBOT's pose
			var offsets = constants.robotToCamera;
			visionPose = cameraPose.plus(offsets.inverse()).toPose2d();

			// Even in multi-tag, we check the distance to the primary target
			double bestTargetDist =
					result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();

			xStdev = 0.05;
			yStdev = 0.05;

			// Condition: Distance <= 2.5m
			if (bestTargetDist <= 2.5) {
				thetaStdev = Units.degreesToRadians(0.1); // Trust vision rotation
			} else {
				thetaStdev = Double.POSITIVE_INFINITY; // Trust Gyro rotation
			}

		} else {
			// --- SINGLE-TAG CASE ---
			var target = result.getBestTarget();
			double distance = target.getBestCameraToTarget().getTranslation().getNorm();
			double ambiguity = target.getPoseAmbiguity();

			visionPose =
					PhotonUtils.estimateFieldToRobotAprilTag(
									target.getBestCameraToTarget(),
									FieldLayout.APRILTAG_MAP.getTagPose(target.getFiducialId()).get(),
									constants.robotToCamera.inverse())
							.toPose2d();

			// Condition: Distance <= 2.5m AND Ambiguity <= 0.2
			if (distance <= 2.5 && ambiguity <= 0.2) {
				// High trust: Update both Translation and Rotation
				xStdev = 0.1 * Math.pow(distance, 2);
				yStdev = 0.1 * Math.pow(distance, 2);
				thetaStdev = 0.2 * Math.pow(distance, 2);
			} else {
				// Low trust or Far away: Translation only, keep Gyro Rotation
				xStdev = 0.2 * Math.pow(distance, 2);
				yStdev = 0.2 * Math.pow(distance, 2);
				thetaStdev = 99999.0;
				visionPose = new Pose2d(visionPose.getTranslation(), currentGyroRotation);
			}
		}

		// Apply to Estimator
		if (Robot.isReal()) {
			drive.addVisionUpdate(visionPose, timestamp, VecBuilder.fill(xStdev, yStdev, thetaStdev));
		}

		robotField.setRobotPose(visionPose);
		botPose = visionPose;
	}

	private void processFramesRigged(Matrix<N3, N1> riggedness) {
		var result = camera.getLatestResult();

		if (result.hasTargets()) {
			var target = result.getBestTarget();

			var initBotPose =
					PhotonUtils.estimateFieldToRobotAprilTag(
							target.getBestCameraToTarget(),
							FieldLayout.APRILTAG_MAP.getTagPose(target.getFiducialId()).get(),
							constants.robotToCamera.inverse());

			botPose = initBotPose.toPose2d();

			if (Robot.isReal()) {
				drive.addVisionUpdate(botPose, result.getTimestampSeconds(), riggedness);
			}

			robotField.setRobotPose(botPose);
		} else {
			robotField.setRobotPose(Pose2d.kZero);
		}
	}

	public Command bootUpSequence() {
		Matrix<N3, N1> riggedness =
				VecBuilder.fill(
						Math.pow(0.02, 1), // vision
						Math.pow(0.02, 1),
						Math.pow(0.02, 1));

		return Commands.run(() -> processFramesRigged(riggedness))
				.withTimeout(3)
				.andThen(
						() ->
								drive
										.getCtreDrive()
										.setVisionMeasurementStdDevs(VisionConstants.LOCAL_MEASUREMENT_STD_DEVS));
	}

	public void periodic() {
		isConnected = !(Timer.getFPGATimestamp() - latestTimestamp > 1.0);
		processFrames();
	}

	public boolean hasTarget() {
		return hasTarget;
	}

	public boolean isConnected() {
		return isConnected;
	}

	public VisionDeviceConstants getConstants() {
		return constants;
	}

	public PhotonCameraSim getSimulation() {
		return sim;
	}
}
