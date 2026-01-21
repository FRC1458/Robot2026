package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.lib.control.ControlConstants.*;
import frc.robot.subsystems.drive.ctre.CtreDriveConstants;

public final class DriveConstants {		
    public static final double EPSILON_TRANSLATION = 0.02; // cm
    public static final double EPSILON_ROTATION = Units.Degrees.of(2).in(Units.Radians);
    
    // Maximums
    public static final double MAX_SPEED = Units.MetersPerSecond.of(4.5).in(Units.MetersPerSecond);
    public static final double MAX_ACCEL = Units.MetersPerSecondPerSecond.of(9.0).in(Units.MetersPerSecondPerSecond);
    public static final double MAX_ROTATION_SPEED = 
        Units.RotationsPerSecond.of(2.0).in(Units.RadiansPerSecond);
    public static final double MAX_ROTATION_ACCEL = 
        Units.RotationsPerSecondPerSecond.of(4.0).in(Units.RadiansPerSecondPerSecond);

    // Swerve dimensions
    public static final double TRACK_WIDTH = Units.Inches.of(24).in(Units.Meters);
    public static final double WHEEL_BASE =	Units.Inches.of(24).in(Units.Meters);
    public static final double WHEEL_DIAMETER = 2 * CtreDriveConstants.kWheelRadius.in(Units.Meters);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    // Stability constants
    public static final double MAX_VELOCITY_STABLE = 10; // degrees per second
    public static final double MAX_PITCH_STABLE = 5; // degrees
    public static final LinearVelocity MAX_SPEED_SCORING_TRANSLATION =
        Units.Centimeters.of(15.0).per(Units.Seconds);
    public static final AngularVelocity MAX_ROTATION_SPEED_SCORING =
        Units.Degrees.of(7.0).per(Units.Seconds); // oh god
    public static final Time POSE_RESET_PREVENTION_TIME = Units.Seconds.of(0.15);

    // Trajectory and snap constants
    public static final PIDVConstants TRANSLATION_CONSTANTS = 
        new PIDVConstants(5, 0.0, 0.1);
    public static final ProfiledPIDVConstants PROFILED_TRANSLATION_CONSTANTS = 
        new ProfiledPIDVConstants(5.0, 0.0, 0.1, 
            new TrapezoidProfile.Constraints(
                MAX_SPEED, 
                MAX_ACCEL));
    public static final PIDVConstants ROTATION_CONSTANTS = 
        new PIDVConstants(16.0, 0.0, 0.1);
    public static final ProfiledPIDVConstants PROFILED_ROTATION_CONSTANTS = 
        new ProfiledPIDVConstants(16.0, 0.0, 0.1, 
            new TrapezoidProfile.Constraints(
                MAX_ROTATION_SPEED, 
                MAX_ROTATION_ACCEL));
    public static final double ACCELERATION_CONSTANT = 0.1;

    public static enum FieldPoses {
        HUB(new Pose2d(4.034536, 4.625594, Rotation2d.kZero));
        public Pose2d pose;
        private FieldPoses(Pose2d pose) {
            this.pose = pose;
        }
    }
}
