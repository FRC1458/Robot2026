package frc.robot.subsystems.drive;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.lib.control.ControlConstants.PIDFConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.subsystems.drive.ctre.CtreDriveConstants;

public final class DriveConstants {		
    // Maximums
    public static final double MAX_SPEED = 5.0;
    public static final double MAX_ACCEL = 5.0;
    public static final double MAX_ROTATION_SPEED = 
        Units.DegreesPerSecond.of(540.0).in(Units.RadiansPerSecond);
    public static final double MAX_ROTATION_ACCEL = 
        Units.DegreesPerSecond.of(2880.0).in(Units.RadiansPerSecond);

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
    public static final PIDFConstants TRANSLATION_CONSTANTS = 
        new PIDFConstants(3.5, 0.0, 0.1, 1.0);
    public static final ProfiledPIDFConstants PROFILED_TRANSLATION_CONSTANTS = 
        new ProfiledPIDFConstants(3.5, 0.0, 0.1, 1.0, 
            new TrapezoidProfile.Constraints(
                MAX_SPEED * 0.7, 
                MAX_ACCEL * 0.7));
    public static final ProfiledPIDFConstants ROTATION_CONSTANTS = 
        new ProfiledPIDFConstants(4.0, 0.0, 0.0, 1.0, 
            new TrapezoidProfile.Constraints(
                MAX_ROTATION_SPEED, 
                MAX_ROTATION_ACCEL));
    public static final double ACCELERATION_CONSTANT = 0.1;
}
