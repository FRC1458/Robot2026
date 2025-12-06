package frc.robot.lib.control;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.lib.control.ControlConstants.*;

public class PIDVController {
    private final PIDFConstants constants;

    private double positionMeasurement = 0.0;
    private double velocityMeasurement = 0.0;

    private double target = 0.0;
    private double feedforward = 0.0;
    private double integral = 0.0;

    private double error = 0.0;

    private boolean isContinuous = false;
    private double minRange = 0.0;
    private double maxRange = 0.0;

    /**
     * Creates a PIDV controller, which is a PID controller 
     * where the derivative is replaced by accurate velocity measurements.
     * @param constants The {@link PIDFConstants}.
     */
    public PIDVController(PIDFConstants constants) {
        this.constants = constants;
    }

    public PIDVController(PIDConstants constants) {
        this(new PIDFConstants(constants));
    }
    
    /**
     * Makes the controller continuous, which means that values repeat.
     * @param minInput The minimum value.
     * @param maxInput The maximum value.
     */
    public PIDVController enableContinuousInput(double minInput, double maxInput) {
        isContinuous = true;
        minRange = minInput;
        maxRange = maxInput;
        return this;
    }

    /** Makes the controller discontinuous */
    public PIDVController disableContinuousInput() {
        isContinuous = false;
        return this;
    }

    /** Sets the current position and velocity measurement. */
    public PIDVController setMeasurement(double positionMeasurement, double velocityMeasurement) {
        this.positionMeasurement = positionMeasurement;
        this.velocityMeasurement = velocityMeasurement;
        return this;
    }

    /** Sets the goal */
    public PIDVController setTarget(double target) {
        this.target = target;
        return this;
    }

    /** Sets the feedforward value. */
    public PIDVController setFeedforward(double feedforward) {
        this.feedforward = feedforward;
        return this;
    }

    public double getOutput() {
        if (isContinuous) {
            error = MathUtil.inputModulus(
                target - positionMeasurement, 
                -(maxRange - minRange) / 2.0, 
                (maxRange - minRange) / 2.0);
        } else {
            error = target - positionMeasurement;
        }

        integral += error * Constants.DT;

        double derivative = feedforward - velocityMeasurement;

        return constants.kP * error
            + constants.kI * integral
            + constants.kD * derivative
            + constants.kF * feedforward;
    }

    public double getError() {
        return error;
    }

    /** Sets the integral value. */
    public PIDVController setIntegral(double integral) {
        this.integral = integral;
        return this;
    }

    /** Resets the controller. */
    public PIDVController reset() {
        integral = 0.0;
        feedforward = 0.0;
        error = 0.0;
        return this;
    }
}
