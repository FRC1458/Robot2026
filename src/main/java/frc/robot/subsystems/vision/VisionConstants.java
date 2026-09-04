package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;

public class VisionConstants {
	// TODO: this must be tuned to specific robot
	public static final int OBSERVATION_BUFFER_SIZE = 50;
	public static final Matrix<N3, N1> STATE_STD_DEVS =
			VecBuilder.fill(Math.pow(0.02, 1), Math.pow(0.02, 1), Math.pow(0.02, 1)); // drive
	public static final Matrix<N3, N1> LOCAL_MEASUREMENT_STD_DEVS =
			VecBuilder.fill(
					Math.pow(0.35, 1), // vision
					Math.pow(0.35, 1),
					Math.pow(Double.POSITIVE_INFINITY, 1));
	public static final Matrix<N3, N1> ROTATION_STD_DEVS =
			VecBuilder.fill(
					Math.pow(0.35, 1), // vision
					Math.pow(0.35, 1),
					Math.pow(0.35, 1));

	public static enum VisionDeviceConstants {
		FR_CONSTANTS(
				"right", // right camera
				new Transform3d(
						new Translation3d(
								Inches.of(13.124114), // wpi x-axis positive is forward direction
								// Inches.of(12.624114),
								Inches.of(-4.678), // wpi y-axis positive is strafe left, so right camera shall have
								// negative offset
								Inches.of(14.365654)),

						// Rotation3d.kZero
						// new Rotation3d(0, 26 * Constants.TAU / 360.0, -24 * Constants.TAU / 360.0)), //(roll:
						// x, pitch: y, yaw: z)
						// new Rotation3d(0, 26 * Constants.TAU / 360.0, -39.5 * Constants.TAU / 360.0)
						new Rotation3d(Degrees.of(0), Degrees.of(-26), Degrees.of(-20))),
				1,
				1280,
				800),

		FL_CONSTANTS(
				"left", // left camera
				new Transform3d(
						new Translation3d(
								Inches.of(13.262586), // wpi x-axis positive is forward direction
								Inches.of(3.171), // wpi y-axis positive is strafe left, so left camera shall have
								// positive offset
								Inches.of(14.325391)),
						// Rotation3d.kZero
						new Rotation3d(Degrees.of(0), Degrees.of(-26), Degrees.of(30))
						// new Rotation3d(0, 26 * Constants.TAU / 360.0, 46.5 * Constants.TAU / 360.0)
						),
				2,
				1280,
				800);

		public final String tableName;
		public final Transform3d robotToCamera;
		public final int cameraId;
		public final int cameraResolutionWidth;
		public final int cameraResolutionHeight;

		private VisionDeviceConstants(
				String tableName,
				Transform3d robotToCamera,
				int cameraId,
				int cameraResolutionWidth,
				int cameraResolutionHeight) {
			this.tableName = tableName;
			this.robotToCamera = robotToCamera;
			this.cameraId = cameraId;
			this.cameraResolutionWidth = cameraResolutionWidth;
			this.cameraResolutionHeight = cameraResolutionHeight;
		}
	}
}
