package frc.robot.subsystems.vision;

import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants.VisionDeviceConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.lib.field.FieldLayout;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;

public class VisionDevice {
	private final VisionDeviceConstants constants;

	public Field2d robotField;
	public PhotonCamera camera;
	// public PhotonPoseEstimator poseEstimator;
	private boolean hasTarget;

	public Pose2d botPose;

	// private double cameraExposure = 20;
	// private boolean cameraAutoExposure = false;
	// private double cameraGain = 10;

	// private long fps = -1;
	private double latestTimestamp = 0.0;
	// private List<VisionFrame> frames = new ArrayList<VisionFrame>();
	private boolean isConnected;
	private PhotonCameraSim sim;


	public VisionDevice(VisionDeviceConstants constants) {
		robotField = new Field2d();
		SmartDashboard.putData("VisionDevice/" + constants.tableName, robotField);

		camera = new PhotonCamera(constants.tableName);

		if (Robot.isSimulation()) {
			SimCameraProperties cameraProp = new SimCameraProperties();
			// A 640 x 480 camera with a 100 degree diagonal FOV.
			cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
			// Approximate detection noise with average and standard deviation error in pixels.
			cameraProp.setCalibError(0.25, 0.08);
			// Set the camera image capture framerate (Note: this is limited by robot loop rate).
			cameraProp.setFPS(20);
			// The average and standard deviation in milliseconds of image data latency.
			cameraProp.setAvgLatencyMs(35);
			cameraProp.setLatencyStdDevMs(5);

			sim = new PhotonCameraSim(camera, cameraProp);
		}

		// poseEstimator = new PhotonPoseEstimator(FieldLayout.APRILTAG_MAP, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, constants.robotToCamera);

		this.constants = constants;
		TelemetryManager.getInstance().addStructPublisher(
			constants.name() + "Pose", Pose2d.struct, 
			() -> botPose);

		hasTarget = false;
		xOffset = new LoggedNetworkNumber(constants.tableName + "xoffset", constants.robotToCamera.getX());;
		yOffset = new LoggedNetworkNumber(constants.tableName + "yoffset", constants.robotToCamera.getY());;
	}

	LoggedNetworkNumber xOffset;
	LoggedNetworkNumber yOffset;

	@SuppressWarnings("removal")
	private void processFrames() {
		var result = camera.getLatestResult();
		if (!result.hasTargets()) {
			robotField.setRobotPose(Pose2d.kZero);
			// System.out.println("No target");
			return;
		}
		
		Pose2d visionPose;
		double xStdev, yStdev, thetaStdev;
		double timestamp = result.getTimestampSeconds();
		
		// Get current state for fallbacks
		Rotation2d currentGyroRotation = Drive.getInstance().getPose().getRotation();

		var multiTagResult = result.getMultiTagResult();
		
		if (multiTagResult.isPresent()) {
			// --- MULTI-TAG CASE ---
			var multiTag = multiTagResult.get();
			Pose3d cameraPose = new Pose3d(
					multiTag.estimatedPose.best.getTranslation(), 
					multiTag.estimatedPose.best.getRotation()
				);
			
			// Adjust for the Camera-to-Robot offset to get the ROBOT's pose
			// We multiply the Camera Pose by the inverse of the Robot-to-Camera transform
			var offsets = constants.robotToCamera;
			offsets = new Transform3d(new Translation3d(
				xOffset.get(),
				yOffset.get(),
				offsets.getZ()
			), offsets.getRotation());
			visionPose = cameraPose.plus(offsets.inverse()).toPose2d();			
			
			// Even in multi-tag, we check the distance to the primary target
			double bestTargetDist = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();

			xStdev = 0.05;
			yStdev = 0.05;

			// Condition: Distance <= 2.5m (Ambiguity is usually ~0 in Multi-Tag), tune it in real field 
			if (bestTargetDist <= 2.5) {
				thetaStdev = Units.degreesToRadians(0.1); // Trust vision rotation
				// System.out.println("closer");
			} else {
				thetaStdev = Double.POSITIVE_INFINITY; // Trust Gyro rotation
				// System.out.println("farther");
				// visionPose = new Pose2d(visionPose.getTranslation(), currentGyroRotation);
			}

		} else {
			// --- SINGLE-TAG CASE ---
			var target = result.getBestTarget();
			double distance = target.getBestCameraToTarget().getTranslation().getNorm();
			double ambiguity = target.getPoseAmbiguity();

			visionPose = PhotonUtils.estimateFieldToRobotAprilTag(
				target.getBestCameraToTarget(),
				FieldLayout.APRILTAG_MAP.getTagPose(target.getFiducialId()).get(),
				constants.robotToCamera.inverse()
			).toPose2d();

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
			// System.out.println("single tag");
		}

		// Apply to Estimator
		if (Robot.isReal()) {
			Drive.getInstance().addVisionUpdate(
				visionPose, 
				timestamp,
				VecBuilder.fill(xStdev, yStdev, thetaStdev)
			);
		}
		
		robotField.setRobotPose(visionPose);
		botPose = visionPose;

		// var results = camera.getAllUnreadResults();
		// for (var result : results) {
		// 	var multiTagResult = result.getMultiTagResult();
		// 	if (multiTagResult.isPresent()) {
		// 		var fieldToCamera = multiTagResult.get().estimatedPose.best;
				
		// 	}
		// }

		// if (Robot.isSimulation()) {
		// 	botPose = sim.process(0.01, constants.robotToCamera);
		// }
		
		// var result = camera.getLatestResult();
		// if (result.hasTargets()) {
		// 	List<PhotonTrackedTarget> targets = result.getTargets();
		// 	double bestAmbig = 0.2;
		// 	double bestDist = 4.0;
		// 	Optional<PhotonTrackedTarget> bestTarget = Optional.empty();
		// 	for (int x = 0; x < targets.size(); x++) {
		// 		if (targets.get(x).getPoseAmbiguity() < bestAmbig && targets.get(0).getBestCameraToTarget().getTranslation().getNorm() < bestDist) {
		// 			bestTarget = Optional.of(targets.get(x));
		// 		}
		// 	}
		// 	if (bestTarget.isPresent()) {
		// 		var target = bestTarget.get();

		// 		var initBotPose = PhotonUtils.estimateFieldToRobotAprilTag(
		// 			target.getBestCameraToTarget(), 
		// 			FieldLayout.APRILTAG_MAP.getTagPose(
		// 				target.getFiducialId()).get(), 
		// 				constants.robotToCamera.inverse());
		// 		// var estimatedPose = poseEstimator.update(result);

		// 		// if (estimatedPose.isEmpty()) {
		// 		// 	botPose = initBotPose.toPose2d();
		// 		// } else {
		// 		// 	botPose = estimatedPose.get().estimatedPose.toPose2d();
		// 		// }

		// 		botPose = initBotPose.toPose2d();
		// 		if (Robot.isReal()) {
		// 			if (result.hasTargets()) {
		// 				Drive.getInstance().addVisionUpdate(botPose, result.getTimestampSeconds());
		// 			}
		// 		}
				
		// 		if (result.hasTargets()) {
		// 			robotField.setRobotPose(botPose);
		// 		} else {
		// 			robotField.setRobotPose(Pose2d.kZero);
		// 		}
		// 		// poseEstimator.setReferencePose(Drive.getInstance().getPose());
		// 	}
		// };

		// int[] validIds = { 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11 };

		// if (result.getBestTarget().getAlternateCameraToTarget().getTranslation().getNorm() < 3
		// 		&& MathUtil.inputModulus(result.getBestTarget().getAlternateCameraToTarget().getRotation().toRotation2d().getDegrees() + 15, -180, 180) < 30
		// 		&& Arrays.stream(validIds).anyMatch(n -> n == (int) result.getBestTarget().getFiducialId())) {
		// 	inSnapRange = true;
		// } else {
		// 	inSnapRange = false;
		// }
	}

	@SuppressWarnings("removal")
	private void processFramesRigged(Matrix<N3, N1> riggedness) {
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			var target = result.getBestTarget();

			var initBotPose = PhotonUtils.estimateFieldToRobotAprilTag(
				target.getBestCameraToTarget(), 
				FieldLayout.APRILTAG_MAP.getTagPose(
					target.getFiducialId()).get(), 
					constants.robotToCamera.inverse());

			botPose = initBotPose.toPose2d();
			if (Robot.isReal()) {
				if (result.hasTargets()) {
					Drive.getInstance().addVisionUpdate(botPose, result.getTimestampSeconds(), riggedness);
				}
			}
			
			if (result.hasTargets()) {
				robotField.setRobotPose(botPose);
			} else {
				robotField.setRobotPose(Pose2d.kZero);
			}
		};
	}

	public Command bootUpSequence() {
		Matrix<N3, N1> riggedness = VecBuilder.fill(
            Math.pow(0.02, 1), // vision
            Math.pow(0.02, 1),
            Math.pow(0.02, 1));
		return Commands.run(() -> processFramesRigged(riggedness))
			.withTimeout(3)
			.andThen(() -> 
				Drive.getInstance().getCtreDrive()
				.setVisionMeasurementStdDevs(
					VisionConstants.LOCAL_MEASUREMENT_STD_DEVS));
	}

	public boolean hasTarget() {
		return hasTarget;
	}

	public void periodic() {
		isConnected = !(Timer.getFPGATimestamp() - latestTimestamp > 1.0);

		processFrames();

		// SmartDashboard.putNumber(
		// 		"Vision " + constants.tableName + "/Last Update Timestamp Timestamp", latestTimestamp);
		// // SmartDashboard.putNumber("Vision " + mConstants.tableName + "/N Queued Updates", frames.size());
		// SmartDashboard.putBoolean("Vision " + constants.tableName + "/is Connnected", isConnected);
	}

	public boolean isConnected() {
		return isConnected;
	}

	// private static class VisionFrame implements Comparable<VisionFrame> {
	// 	double timestamp;

	// 	@Override
	// 	public int compareTo(VisionFrame o) {
	// 		return Double.compare(timestamp, o.timestamp);
	// 	}
	// }

	public VisionDeviceConstants getConstants() {
		return constants;
	}

	public PhotonCameraSim getSimulation() {
		return sim;
	}
}
