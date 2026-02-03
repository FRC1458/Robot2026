package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

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
