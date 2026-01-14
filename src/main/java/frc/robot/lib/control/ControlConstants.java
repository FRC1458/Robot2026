package frc.robot.lib.control;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A class containing controller constants.
 */
public class ControlConstants {
    /**
     * Constants used for a {@link PIDVController}
     */
    public static class PIDVConstants {
        public final double kP;
        public final double kI;
        public final double kD;
        public PIDVConstants(double p, double i, double d) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
        }
    }

    /**
     * Constants used for a {@link ProfiledPIDVController}
     */
    public static class ProfiledPIDVConstants {
        public final double kP;
        public final double kI;
        public final double kD;
        public final TrapezoidProfile.Constraints constraints;
        public ProfiledPIDVConstants(double p, double i, double d, TrapezoidProfile.Constraints constraints) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
            this.constraints = constraints;
        }

        public ProfiledPIDVConstants(PIDVConstants constants, TrapezoidProfile.Constraints constraints) {
            this(constants.kP, constants.kI, constants.kD, constraints);
        }

        public PIDVConstants getPIDVConstants() {
            return new PIDVConstants(kP, kI, kD);
        }
    }
}
