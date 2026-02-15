package frc.robot.lib.field;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveConstants;

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

	public static final List<Pose2d> ENTRY_RIGHT_POSES = new ArrayList<>();
	public static final List<Pose2d> ENTRY_LEFT_POSES = new ArrayList<>();

	public static final HashMap<Pose2d,Pose2d> TARGET_RIGHT_POSES = new HashMap<>();
	public static final HashMap<Pose2d,Pose2d> TARGET_LEFT_POSES = new HashMap<>();
	
	static {
		try {
			APRILTAG_MAP = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
			field = new Field2d();
			SmartDashboard.putData(field);
			int[] rightTrenchIds = {1,6,23,28};
			int[] leftTrenchIds = {7,12,17,22};
			Transform2d moveLeft = new Transform2d(
				-Units.inchesToMeters(45), 0, Rotation2d.kZero);
			Transform2d moveRight = new Transform2d(
				Units.inchesToMeters(45),0,Rotation2d.kZero);
			Transform2d turn = new Transform2d(0,0,Rotation2d.kPi);
			for (int i : rightTrenchIds) {
				var tagPose = APRILTAG_MAP.getTagPose(i).get().toPose2d();
				ENTRY_RIGHT_POSES.add(tagPose.transformBy(moveLeft));
				TARGET_RIGHT_POSES.put(tagPose.transformBy(moveLeft),tagPose.transformBy(moveRight));
			}
			for (int i : leftTrenchIds) {
				var tagPose = APRILTAG_MAP.getTagPose(i).get().toPose2d();
				ENTRY_LEFT_POSES.add(tagPose.transformBy(moveLeft));
				TARGET_LEFT_POSES.put(tagPose.transformBy(moveLeft),tagPose.transformBy(moveRight));
			}
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

	public static APTarget getTrenchEntry(Pose2d pose) {
		var set = isLeftOfTrench(pose) ? ENTRY_LEFT_POSES : ENTRY_RIGHT_POSES;
		var out = Collections.min(
			set,
			Comparator.comparing(
				(Pose2d other) -> pose.getTranslation().getDistance(other.getTranslation())
			).thenComparing(
				(Pose2d other) ->
				Math.abs(pose.getRotation().minus(other.getRotation()).getRadians())));
		return new APTarget(out).withEntryAngle(out.getRotation());
	}

	public static APTarget getTrenchTarget(Pose2d pose) {
		var set = isLeftOfTrench(pose) ? TARGET_LEFT_POSES : TARGET_RIGHT_POSES;
		var out = set.get(getTrenchEntry(pose).getReference());
		return new APTarget(out).withEntryAngle(out.getRotation());
	}

	public static boolean isLeftOfTrench(Pose2d pose) {
		double x = pose.getX();
		return x < 4.63 || (x > 8.3 && x < 11.95); //TODO: correct values
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