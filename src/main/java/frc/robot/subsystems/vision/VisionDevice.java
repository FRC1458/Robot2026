package frc.robot.subsystems.vision;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants.VisionDeviceConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.lib.field.FieldLayout;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

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

		hasTarget = false;
	}

	private void processFrames() {
		// var results = camera.getAllUnreadResults();
		// for (var result : results) {
		// 	var multiTagResult = result.getMultiTagResult();
		// 	if (multiTagResult.isPresent()) {
		// 		var fieldToCamera = multiTagResult.get().estimatedPose.best;
				
		// 	}
		// }

		
		var result = camera.getLatestResult();
		if (result.hasTargets()) {
			var target = result.getBestTarget();

			var initBotPose = PhotonUtils.estimateFieldToRobotAprilTag(
				target.getBestCameraToTarget(), 
				FieldLayout.APRILTAG_MAP.getTagPose(
					target.getFiducialId()).get(), 
					constants.robotToCamera.inverse());
			// var estimatedPose = poseEstimator.update(result);

			// if (estimatedPose.isEmpty()) {
			// 	botPose = initBotPose.toPose2d();
			// } else {
			// 	botPose = estimatedPose.get().estimatedPose.toPose2d();
			// }

			botPose = initBotPose.toPose2d();
			if (Robot.isReal()) {
				if (result.hasTargets()) {
					Drive.getInstance().addVisionUpdate(botPose, result.getTimestampSeconds());
				}
			}
			
			if (result.hasTargets()) {
				robotField.setRobotPose(botPose);
			} else {
				robotField.setRobotPose(Pose2d.kZero);
			}
			// poseEstimator.setReferencePose(Drive.getInstance().getPose());

		};

		// int[] validIds = { 17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11 };

		// if (result.getBestTarget().getAlternateCameraToTarget().getTranslation().getNorm() < 3
		// 		&& MathUtil.inputModulus(result.getBestTarget().getAlternateCameraToTarget().getRotation().toRotation2d().getDegrees() + 15, -180, 180) < 30
		// 		&& Arrays.stream(validIds).anyMatch(n -> n == (int) result.getBestTarget().getFiducialId())) {
		// 	inSnapRange = true;
		// } else {
		// 	inSnapRange = false;
		// }
	}

	public boolean hasTarget() {
		return hasTarget;
	}

	public void periodic() {
		isConnected = !(Timer.getFPGATimestamp() - latestTimestamp > 1.0);

		processFrames();

		SmartDashboard.putNumber(
				"Vision " + constants.tableName + "/Last Update Timestamp Timestamp", latestTimestamp);
		// SmartDashboard.putNumber("Vision " + mConstants.tableName + "/N Queued Updates", frames.size());
		SmartDashboard.putBoolean("Vision " + constants.tableName + "/is Connnected", isConnected);
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
