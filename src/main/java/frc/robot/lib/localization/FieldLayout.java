package frc.robot.lib.localization;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	//TODO: this must be tuned to the specific year's field
	public static Field2d field;
	public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
	public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);

	public static final double APRITAG_WIDTH = Units.inchesToMeters(6.50);
	public static final AprilTagFieldLayout APRILTAG_MAP;

	public static final List<Pose2d> ALIGN_POSES_RIGHT = new ArrayList<>();
	public static final List<Pose2d> ALIGN_POSES_LEFT = new ArrayList<>();

	static {
		try {
			APRILTAG_MAP = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
			field = new Field2d();
			SmartDashboard.putData(field);
			int[] reefIds = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
			Transform2d leftReefTransform = new Transform2d(
				Constants.Drive.TRACK_WIDTH / 2 + Units.inchesToMeters(1.5), -Units.inchesToMeters(13.0 / 2), Rotation2d.kPi);
			for (int i : reefIds) {
				ALIGN_POSES_LEFT.add(APRILTAG_MAP.getTagPose(i).get().toPose2d().transformBy(leftReefTransform));
			}
			Transform2d rightReefTransform = new Transform2d(
				Constants.Drive.TRACK_WIDTH / 2 + Units.inchesToMeters(1.5), Units.inchesToMeters(13.0 / 2), Rotation2d.kPi);
			for (int i : reefIds) {
				ALIGN_POSES_RIGHT.add(APRILTAG_MAP.getTagPose(i).get().toPose2d().transformBy(rightReefTransform));
			}
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
            blue_pose = new Pose2d(
                new Translation2d((FIELD_LENGTH / 2.0) + ((FIELD_LENGTH / 2.0) - blue_pose.getX()), blue_pose.getY()), 
                new Rotation2d(-blue_pose.getRotation().getCos(), blue_pose.getRotation().getSin())
            );
		}
		return blue_pose;
	}

	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
            blue_translation = new Translation2d((FIELD_LENGTH / 2.0) + ((FIELD_LENGTH / 2.0) - blue_translation.getX()), blue_translation.getY());
		}
		return blue_translation;
	}

	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = new Rotation2d(-blue_rotation.getCos(), blue_rotation.getSin());
		}
		return blue_rotation;
	}

	public static double distanceFromAllianceWall(double x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return FIELD_LENGTH - x_coordinate;
		}
		return x_coordinate;
	}
}