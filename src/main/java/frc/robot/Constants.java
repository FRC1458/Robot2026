package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.field.FieldLayout;
import frc.robot.lib.field.FieldUtil;
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
		public static final PathConstraints GLOBAL_CONSTRAINTS = new PathConstraints(DriveConstants.MAX_SPEED * 0.85,
				DriveConstants.MAX_ACCEL * 0.85,
				DriveConstants.MAX_ROTATION_SPEED * 0.85,
				DriveConstants.MAX_ROTATION_ACCEL * 0.85);
		public static final double GENERATION_WAIT_TIME = 5;
	}

	// Copyright (c) 2025-2026 Littleton Robotics
	// http://github.com/Mechanical-Advantage
	//
	// Use of this source code is governed by an MIT-style
	// license that can be found in the LICENSE file at
	// the root directory of this project.
	/**
	 * Contains information for location of field element and other useful reference
	 * points.
	 *
	 * <p>
	 * NOTE: All constants are defined relative to the field coordinate system, and
	 * from the
	 * perspective of the blue alliance station
	 */
	public static class FieldConstants {
		public static Translation3d allianceCorrected(Translation3d t) {
			if (DriverStation.getAlliance().isPresent()) {
				Alliance a = DriverStation.getAlliance().orElseThrow();
				if (a == Alliance.Red) {
					var t2d = FieldUtil.flipTranslation(t.toTranslation2d());
					return new Translation3d(t2d.getX(), t2d.getY(), t.getZ());
				} else {
					return t;
				}
			} else {
				return t;
			}
		}

		// AprilTag related constants
		public static final int aprilTagCount = FieldLayout.APRILTAG_MAP.getTags().size();
		public static final double aprilTagWidth = edu.wpi.first.math.util.Units.inchesToMeters(6.5);

		// Field dimensions
		public static final double fieldLength = FieldLayout.APRILTAG_MAP.getFieldLength();
		public static final double fieldWidth = FieldLayout.APRILTAG_MAP.getFieldWidth();

		// Fuel dimensions
		public static final double fuelDiameter = edu.wpi.first.math.util.Units.inchesToMeters(5.91);

		/**
		 * Officially defined and relevant vertical lines found on the field (defined by
		 * X-axis offset)
		 */
		public static class LinesVertical {
			public static final double center = fieldLength / 2.0;
			public static final double starting = FieldLayout.APRILTAG_MAP.getTagPose(26).get().getX();
			public static final double allianceZone = starting;
			public static final double hubCenter = FieldLayout.APRILTAG_MAP.getTagPose(26).get().getX()
					+ Hub.width / 2.0;
			public static final double neutralZoneNear = center - edu.wpi.first.math.util.Units.inchesToMeters(120);
			public static final double neutralZoneFar = center + edu.wpi.first.math.util.Units.inchesToMeters(120);
			public static final double oppHubCenter = FieldLayout.APRILTAG_MAP.getTagPose(4).get().getX()
					+ Hub.width / 2.0;
			public static final double oppAllianceZone = FieldLayout.APRILTAG_MAP.getTagPose(10).get().getX();
		}

		/**
		 * Officially defined and relevant horizontal lines found on the field (defined
		 * by Y-axis offset)
		 *
		 * <p>
		 * NOTE: The field element start and end are always left to right from the
		 * perspective of the
		 * alliance station
		 */
		public static class LinesHorizontal {

			public static final double center = fieldWidth / 2.0;

			// Right of hub
			public static final double rightBumpStart = Hub.nearRightCorner.getY();
			public static final double rightBumpEnd = rightBumpStart - RightBump.width;
			public static final double rightBumpMiddle = (rightBumpStart + rightBumpEnd) / 2.0;
			public static final double rightTrenchOpenStart = rightBumpEnd
					- edu.wpi.first.math.util.Units.inchesToMeters(12.0);
			public static final double rightTrenchOpenEnd = 0;

			// Left of hub
			public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
			public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
			public static final double leftBumpMiddle = (leftBumpStart + leftBumpEnd) / 2.0;
			public static final double leftTrenchOpenEnd = leftBumpStart
					+ edu.wpi.first.math.util.Units.inchesToMeters(12.0);
			public static final double leftTrenchOpenStart = fieldWidth;
		}

		/** Hub related constants */
		public static class Hub {
			// Dimensions
			public static final double width = edu.wpi.first.math.util.Units.inchesToMeters(46.0);
			public static final double height = edu.wpi.first.math.util.Units.inchesToMeters(72.0); // includes the
																									// catcher at the
																									// top
			public static final double innerWidth = edu.wpi.first.math.util.Units.inchesToMeters(41.7);
			public static final double innerHeight = edu.wpi.first.math.util.Units.inchesToMeters(56.5);

			// Relevant reference points on alliance side
			public static final Translation3d topCenterPoint = new Translation3d(
					FieldLayout.APRILTAG_MAP.getTagPose(26).get().getX() + width / 2.0,
					fieldWidth / 2.0,
					height);
			public static final Translation3d innerCenterPoint = new Translation3d(
					FieldLayout.APRILTAG_MAP.getTagPose(26).get().getX() + width / 2.0,
					fieldWidth / 2.0,
					innerHeight);

			public static final Translation2d nearLeftCorner = new Translation2d(topCenterPoint.getX() - width / 2.0,
					fieldWidth / 2.0 + width / 2.0);
			public static final Translation2d nearRightCorner = new Translation2d(topCenterPoint.getX() - width / 2.0,
					fieldWidth / 2.0 - width / 2.0);
			public static final Translation2d farLeftCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
					fieldWidth / 2.0 + width / 2.0);
			public static final Translation2d farRightCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
					fieldWidth / 2.0 - width / 2.0);

			// Relevant reference points on the opposite side
			public static final Translation3d oppTopCenterPoint = new Translation3d(
					FieldLayout.APRILTAG_MAP.getTagPose(4).get().getX() + width / 2.0,
					fieldWidth / 2.0,
					height);
			public static final Translation2d oppNearLeftCorner = new Translation2d(
					oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
			public static final Translation2d oppNearRightCorner = new Translation2d(
					oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
			public static final Translation2d oppFarLeftCorner = new Translation2d(
					oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
			public static final Translation2d oppFarRightCorner = new Translation2d(
					oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

			// Hub faces
			public static final Pose2d nearFace = FieldLayout.APRILTAG_MAP.getTagPose(26).get().toPose2d();
			public static final Pose2d farFace = FieldLayout.APRILTAG_MAP.getTagPose(20).get().toPose2d();
			public static final Pose2d rightFace = FieldLayout.APRILTAG_MAP.getTagPose(18).get().toPose2d();
			public static final Pose2d leftFace = FieldLayout.APRILTAG_MAP.getTagPose(21).get().toPose2d();
		}

		/** Left Bump related constants */
		public static class LeftBump {

			// Dimensions
			public static final double width = edu.wpi.first.math.util.Units.inchesToMeters(73.0);
			public static final double height = edu.wpi.first.math.util.Units.inchesToMeters(6.513);
			public static final double depth = edu.wpi.first.math.util.Units.inchesToMeters(44.4);

			// Relevant reference points on alliance side
			public static final Translation2d nearLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
					edu.wpi.first.math.util.Units.inchesToMeters(255));
			public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
			public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
					edu.wpi.first.math.util.Units.inchesToMeters(255));
			public static final Translation2d farRightCorner = Hub.farLeftCorner;

			// Relevant reference points on opposing side
			public static final Translation2d oppNearLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
					edu.wpi.first.math.util.Units.inchesToMeters(255));
			public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
			public static final Translation2d oppFarLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
					edu.wpi.first.math.util.Units.inchesToMeters(255));
			public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
		}

		/** Right Bump related constants */
		public static class RightBump {
			// Dimensions
			public static final double width = edu.wpi.first.math.util.Units.inchesToMeters(73.0);
			public static final double height = edu.wpi.first.math.util.Units.inchesToMeters(6.513);
			public static final double depth = edu.wpi.first.math.util.Units.inchesToMeters(44.4);

			// Relevant reference points on alliance side
			public static final Translation2d nearLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
					edu.wpi.first.math.util.Units.inchesToMeters(255));
			public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
			public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
					edu.wpi.first.math.util.Units.inchesToMeters(255));
			public static final Translation2d farRightCorner = Hub.farLeftCorner;

			// Relevant reference points on opposing side
			public static final Translation2d oppNearLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
					edu.wpi.first.math.util.Units.inchesToMeters(255));
			public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
			public static final Translation2d oppFarLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
					edu.wpi.first.math.util.Units.inchesToMeters(255));
			public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
		}

		/** Left Trench related constants */
		public static class LeftTrench {
			// Dimensions
			public static final double width = edu.wpi.first.math.util.Units.inchesToMeters(65.65);
			public static final double depth = edu.wpi.first.math.util.Units.inchesToMeters(47.0);
			public static final double height = edu.wpi.first.math.util.Units.inchesToMeters(40.25);
			public static final double openingWidth = edu.wpi.first.math.util.Units.inchesToMeters(50.34);
			public static final double openingHeight = edu.wpi.first.math.util.Units.inchesToMeters(22.25);

			// Relevant reference points on alliance side
			public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter, fieldWidth,
					openingHeight);
			public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter,
					fieldWidth - openingWidth, openingHeight);

			// Relevant reference points on opposing side
			public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter,
					fieldWidth, openingHeight);
			public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter,
					fieldWidth - openingWidth, openingHeight);
		}

		public static class RightTrench {

			// Dimensions
			public static final double width = edu.wpi.first.math.util.Units.inchesToMeters(65.65);
			public static final double depth = edu.wpi.first.math.util.Units.inchesToMeters(47.0);
			public static final double height = edu.wpi.first.math.util.Units.inchesToMeters(40.25);
			public static final double openingWidth = edu.wpi.first.math.util.Units.inchesToMeters(50.34);
			public static final double openingHeight = edu.wpi.first.math.util.Units.inchesToMeters(22.25);

			// Relevant reference points on alliance side
			public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter, openingWidth,
					openingHeight);
			public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter, 0,
					openingHeight);

			// Relevant reference points on opposing side
			public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter,
					openingWidth, openingHeight);
			public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter, 0,
					openingHeight);
		}

		/** Tower related constants */
		public static class Tower {
			// Dimensions
			public static final double width = edu.wpi.first.math.util.Units.inchesToMeters(49.25);
			public static final double depth = edu.wpi.first.math.util.Units.inchesToMeters(45.0);
			public static final double height = edu.wpi.first.math.util.Units.inchesToMeters(78.25);
			public static final double innerOpeningWidth = edu.wpi.first.math.util.Units.inchesToMeters(32.250);
			public static final double frontFaceX = edu.wpi.first.math.util.Units.inchesToMeters(43.51);

			public static final double uprightHeight = edu.wpi.first.math.util.Units.inchesToMeters(72.1);

			// Rung heights from the floor
			public static final double lowRungHeight = edu.wpi.first.math.util.Units.inchesToMeters(27.0);
			public static final double midRungHeight = edu.wpi.first.math.util.Units.inchesToMeters(45.0);
			public static final double highRungHeight = edu.wpi.first.math.util.Units.inchesToMeters(63.0);

			// Relevant reference points on alliance side
			public static final Translation2d centerPoint = new Translation2d(
					frontFaceX, FieldLayout.APRILTAG_MAP.getTagPose(31).get().getY());
			public static final Translation2d leftUpright = new Translation2d(
					frontFaceX,
					(FieldLayout.APRILTAG_MAP.getTagPose(31).get().getY())
							+ innerOpeningWidth / 2
							+ edu.wpi.first.math.util.Units.inchesToMeters(0.75));
			public static final Translation2d rightUpright = new Translation2d(
					frontFaceX,
					(FieldLayout.APRILTAG_MAP.getTagPose(31).get().getY())
							- innerOpeningWidth / 2
							- edu.wpi.first.math.util.Units.inchesToMeters(0.75));

			// Relevant reference points on opposing side
			public static final Translation2d oppCenterPoint = new Translation2d(
					fieldLength - frontFaceX,
					FieldLayout.APRILTAG_MAP.getTagPose(15).get().getY());
			public static final Translation2d oppLeftUpright = new Translation2d(
					fieldLength - frontFaceX,
					(FieldLayout.APRILTAG_MAP.getTagPose(15).get().getY())
							+ innerOpeningWidth / 2
							+ edu.wpi.first.math.util.Units.inchesToMeters(0.75));
			public static final Translation2d oppRightUpright = new Translation2d(
					fieldLength - frontFaceX,
					(FieldLayout.APRILTAG_MAP.getTagPose(15).get().getY())
							- innerOpeningWidth / 2
							- edu.wpi.first.math.util.Units.inchesToMeters(0.75));
		}

		public static class Depot {
			// Dimensions
			public static final double width = edu.wpi.first.math.util.Units.inchesToMeters(42.0);
			public static final double depth = edu.wpi.first.math.util.Units.inchesToMeters(27.0);
			public static final double height = edu.wpi.first.math.util.Units.inchesToMeters(1.125);
			public static final double distanceFromCenterY = edu.wpi.first.math.util.Units.inchesToMeters(75.93);

			// Relevant reference points on alliance side
			public static final Translation3d depotCenter = new Translation3d(depth,
					(fieldWidth / 2) + distanceFromCenterY, height);
			public static final Translation3d leftCorner = new Translation3d(depth,
					(fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
			public static final Translation3d rightCorner = new Translation3d(depth,
					(fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
		}

		public static class Outpost {
			// Dimensions
			public static final double width = edu.wpi.first.math.util.Units.inchesToMeters(31.8);
			public static final double openingDistanceFromFloor = edu.wpi.first.math.util.Units.inchesToMeters(28.1);
			public static final double height = edu.wpi.first.math.util.Units.inchesToMeters(7.0);

			// Relevant reference points on alliance side
			public static final Translation2d centerPoint = new Translation2d(0,
					FieldLayout.APRILTAG_MAP.getTagPose(29).get().getY());
		}
	}
}
