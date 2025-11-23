package frc.robot.lib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.control.ControlConstants.*;

/**
 * A profiled version of {@link PIDVController} that uses a TrapezoidProfile
 * to smoothly reach the target position with velocity and acceleration limits.
 * The derivative term is based on velocity feedback (like PIDV).
 */
public class ProfiledPIDVController {
    private final PIDFConstants constants;
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile profile;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private double positionMeasurement = 0.0;
    private double velocityMeasurement = 0.0;
    private double feedforward = 0.0;
    private double integral = 0.0;

    public double error = 0.0;

    private boolean isContinuous = false;
    private double minRange = 0.0;
    private double maxRange = 0.0;

    private final Timer timer = new Timer();

    /**
     * Creates a profiled PIDV controller.
     * @param constants The {@link PIDFConstants}.
     * @param constraints The trapezoidal motion constraints (max velocity and acceleration).
     */
    public ProfiledPIDVController(ProfiledPIDFConstants constants) {
        this.constants = new PIDFConstants(
            constants.kP,
            constants.kI,
            constants.kD,
            constants.kF);
        this.constraints = constants.constraints;
        this.profile = new TrapezoidProfile(constraints);
        timer.start();
    }

    /** Enables continuous input. */
    public void enableContinuousInput(double minInput, double maxInput) {
        isContinuous = true;
        minRange = minInput;
        maxRange = maxInput;
    }

    /** Disables continuous input. */
    public void disableContinuousInput() {
        isContinuous = false;
    }

    /** Sets the current position and velocity measurement. */
    public void setInput(double positionMeasurement, double velocityMeasurement) {
        this.positionMeasurement = positionMeasurement;
        this.velocityMeasurement = velocityMeasurement;
    }

    /** Sets the goal position for the profile (end velocity = 0). */
    public void setTarget(double target) {
        this.goal = new TrapezoidProfile.State(target, 0.0);
    }

    /** Sets the goal with both position and end velocity. */
    public void setTarget(double position, double velocity) {
        this.goal = new TrapezoidProfile.State(position, velocity);
    }

    /** Sets the feedforward value. */
    public void setFeedforward(double feedforward) {
        this.feedforward = feedforward;
    }

    /**
     * Computes the controller output using the current profile and measurements.
     * @return Control output (PID + feedforward).
     */
    public double getOutput() {
        double dt = timer.get();
        timer.reset();
        if (dt <= 0.0) {
            return 0.0;
        }

        setpoint = profile.calculate(dt, goal, setpoint);

        double targetPosition = setpoint.position;
        double targetVelocity = setpoint.velocity;

        error = isContinuous
            ? MathUtil.inputModulus(targetPosition - positionMeasurement, -(maxRange - minRange) / 2.0, (maxRange - minRange) / 2.0)
            : targetPosition - positionMeasurement;

        integral += error * dt;

        double derivative = targetVelocity - velocityMeasurement;

        return constants.kP * error
            + constants.kI * integral
            + constants.kD * derivative
            + constants.kF * feedforward;
    }

    /** Sets the integral term directly. */
    public void setIntegral(double integral) {
        this.integral = integral;
    }

    /** Resets the controller state. */
    public void reset() {
        integral = 0.0;
        feedforward = 0.0;
        error = 0.0;
        setpoint = new TrapezoidProfile.State(positionMeasurement, velocityMeasurement);
        timer.reset();
        timer.start();
    }

    /** Returns the current motion profile setpoint. */
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    /** Returns the goal state. */
    public TrapezoidProfile.State getGoal() {
        return goal;
    }

    /** Updates motion constraints. */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        this.constraints = constraints;
    }
}
