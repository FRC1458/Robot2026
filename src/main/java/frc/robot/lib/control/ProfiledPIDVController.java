package frc.robot.lib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.lib.control.ControlConstants.*;

/** Note: broken do not use rn */
public class ProfiledPIDVController {
	private final PIDFConstants constants;
	private TrapezoidProfile.Constraints constraints;
	private TrapezoidProfile profile;

	private TrapezoidProfile.State goal = new TrapezoidProfile.State();
	private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

	private double positionMeasurement = 0.0;
	private double velocityMeasurement = 0.0;
	private double feedforward = 0.0;
	private boolean isFeedforwardSet = false;
	private double integral = 0.0;

	public double error = 0.0;

	private boolean isContinuous = false;
	private double minRange = 0.0;
	private double maxRange = 0.0;

	private double positionTolerance = 0.01;
	private double velocityTolerance = Double.POSITIVE_INFINITY;
    /**
	 * Creates a Profiled PIDV controller, which is a PID controller 
	 * where the derivative is replaced by accurate velocity measurements,
	 * and it's also profiled
	 * @param constants The {@link ProfiledPIDFConstants}.
	 */
	public ProfiledPIDVController(ProfiledPIDFConstants constants) {
		this.constants = new PIDFConstants(constants.kP, constants.kI, constants.kD, constants.kF);
		this.constraints = constants.constraints;
		this.profile = new TrapezoidProfile(constraints);
	}

    /**
     * Makes the controller continuous, which means that values repeat.
     * @param minInput The minimum value.
     * @param maxInput The maximum value.
     */
	public ProfiledPIDVController enableContinuousInput(double minInput, double maxInput) {
		isContinuous = true;
		minRange = minInput;
		maxRange = maxInput;
		return this;
	}

    /** Makes the controller discontinuous */
	public ProfiledPIDVController disableContinuousInput() {
		isContinuous = false;
		return this;
	}

	/** Sets the current position and velocity measurement. */
    public ProfiledPIDVController setMeasurement(double positionMeasurement, double velocityMeasurement) {
		this.positionMeasurement = positionMeasurement;
		this.velocityMeasurement = velocityMeasurement;
		return this;
	}

	/** Sets the goal */
    public ProfiledPIDVController setTarget(double target) {
		this.goal = new TrapezoidProfile.State(target, 0.0);
		return this;
	}

	/** Sets the goal */
    public void setTarget(double position, double velocity) {
		this.goal = new TrapezoidProfile.State(position, velocity);
	}

	/** Sets the feedforward value. */
    public ProfiledPIDVController setFeedforward(double feedforward) {
		this.feedforward = feedforward;
		isFeedforwardSet = true;
		return this;
	}

	public double getOutput() {
		if (isContinuous) {
			double errorBound = (maxRange - minRange) / 2.0;
			double goalMinDistance = MathUtil.inputModulus(
				goal.position - positionMeasurement, -errorBound, errorBound);
			double setpointMinDistance = MathUtil.inputModulus(
				setpoint.position - positionMeasurement, -errorBound, errorBound);
			goal.position = goalMinDistance + positionMeasurement;
			setpoint.position = setpointMinDistance + positionMeasurement;
		}

		setpoint = profile.calculate(Constants.DT, setpoint, goal);

		double targetPosition = setpoint.position;
		double targetVelocity = setpoint.velocity;

		error = targetPosition - positionMeasurement;
		integral += error * Constants.DT;
		double derivative = targetVelocity - velocityMeasurement;
		
		if (!isFeedforwardSet) {
			feedforward = targetVelocity;
		}
		isFeedforwardSet = false;

		return constants.kP * error
			+ constants.kI * integral
			+ constants.kD * derivative
			+ constants.kF * feedforward;
	}

    /** Sets the integral value. */
	public ProfiledPIDVController setIntegral(double integral) {
		this.integral = integral;
		return this;
	}

    /** Resets the controller. */
	public ProfiledPIDVController reset() {
		integral = 0.0;
		feedforward = 0.0;
		error = 0.0;
		setpoint = new TrapezoidProfile.State(positionMeasurement, velocityMeasurement);
		return this;
	}

	public TrapezoidProfile.State getSetpoint() {
		return setpoint;
	}

	public TrapezoidProfile.State getGoal() {
		return goal;
	}

	public double getError() {
		return error;
	}

	public ProfiledPIDVController setConstraints(TrapezoidProfile.Constraints constraints) {
		this.constraints = constraints;
		this.profile = new TrapezoidProfile(constraints);
		return this;
	}

	public ProfiledPIDVController setTolerance(double positionTolerance, double velocityTolerance) {
		this.positionTolerance = positionTolerance;
		this.velocityTolerance = velocityTolerance;
		return this;
	}

	public ProfiledPIDVController setTolerance(double positionTolerance) {
		return setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
	}

	public boolean atSetpoint() {
		return Math.abs(error) < positionTolerance && Math.abs(setpoint.velocity) < velocityTolerance;
	}

	public boolean atGoal() {
		return atSetpoint() && goal.equals(setpoint);
	}
}
