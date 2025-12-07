package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * Miscallenous constants
 */
public final class Constants {
	public static final double DT = 0.02;
	public static final double EPSILON = 1e-6;
	public static final double LONG_CANT_TIMEOUT_MS = 0;

	public static final double TAU = Math.PI * 2;

	public static final class Controllers {
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final double DRIVER_DEADBAND = 0.07;
	}

	public static final class Odometry {
		public static final int OBSERVATION_BUFFER_SIZE = 50;
		public static final Matrix<N2, N1> STATE_STD_DEVS = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1)); // drive
		public static final Matrix<N2, N1> LOCAL_MEASUREMENT_STD_DEVS = VecBuilder.fill(
				Math.pow(0.02, 1), // vision
				Math.pow(0.02, 1));
	}
	public static final class Limelight { //TODO: this must be tuned to specific robot
		public static enum VisionDeviceConstants {
			// L_CONSTANTS (
			// 	"limelight-left",
			// 	new Transform2d(
			// 		new Translation2d(Units.Inches.of(10.5), Units.Inches.of(1.23)),
			// 		Rotation2d.fromDegrees(-90)),
			// 	0, 1600, 1200),
			
			// R_CONSTANTS (
			// 	"limelight-right",
			// 	new Transform2d(
            //         new Translation2d(Units.Inches.of(10.78), Units.Inches.of(2)),
            //         Rotation2d.fromDegrees(90)),
			// 	0, 1600, 1200),

			// F_CONSTANTS (
			// 	"limelight-front",
			// 	new Transform2d(
			// 		new Translation2d(Units.Inches.of(11.11), Units.Inches.of(4.28)),
			// 		Rotation2d.fromDegrees(0)),
			// 	0, 1600, 1200),

			// B_CONSTANTS (
			// 	"limelight-back",
			// 	new Transform2d(
			// 		new Translation2d(Units.Inches.of(0), Units.Inches.of(-0.96)),
			// 		Rotation2d.fromDegrees(180)),
			// 	0, 1600, 1200);

			// R_CONSTANTS (
			// 	"right",  
			// 	new edu.wpi.first.math.geometry.Transform3d(
			// 		new Translation3d(0.267, 0.0312, 0.2791),
			//		new Rotation3d(TAU/4, -TAU/4, 0)),
			//	0, 1280, 800),
			
			FR_CONSTANTS (
				"frontr",
				new edu.wpi.first.math.geometry.Transform3d(
					new Translation3d(0.2822, 0.1087, 0.1984),
					new Rotation3d(0.5 * TAU, 14.0 * TAU / 360.0, -26.0 * TAU/360.0)),
				1, 1280, 800),
			
			FL_CONSTANTS (
				"frontl",
				new edu.wpi.first.math.geometry.Transform3d(
					new Translation3d(0.2822, -0.1087, 0.1984),
					new Rotation3d(0.5 * TAU, 14.0 * TAU / 360.0, 26.0 * TAU/360.0)),
				2, 1280, 800);

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
				int cameraResolutionHeight
			) {
				this.tableName = tableName;
				this.robotToCamera = robotToCamera;
				this.cameraId = cameraId;
				this.cameraResolutionWidth = cameraResolutionWidth;
				this.cameraResolutionHeight = cameraResolutionHeight;
			}
		}
    }

	public static final class Pathplanner {
		public static RobotConfig config;
		static {
			try {
				config = RobotConfig.fromGUISettings();
			} catch (Exception e) {
				DriverStation.reportError("Pathplanner configs failed to load ", e.getStackTrace());
			}
		}
		public static final PathConstraints GLOBAL_CONSTRAINTS = 
			new PathConstraints(DriveConstants.MAX_SPEED * 0.85, 
				DriveConstants.MAX_ACCEL * 0.85, 
				DriveConstants.MAX_ROTATION_SPEED * 0.85, 
				DriveConstants.MAX_ROTATION_ACCEL * 0.85);
		public static final double GENERATION_WAIT_TIME = 5;
	}
}
