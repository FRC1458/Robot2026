package frc.robot.lib.control;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.lib.control.ControlConstants.*;

public class PIDVController {
    final PIDVConstants constants;

    double positionMeasurement = 0.0;
    double velocityMeasurement = 0.0;

    double targetPosition = 0.0;
    double targetSpeed = 0.0;
    double integral = 0.0;

    double error = 0.0;

    boolean isContinuous = false;
    double minRange = 0.0;
    double maxRange = 0.0;

    /**
     * Creates a PIDV controller, which is a PID controller 
     * where the derivative is replaced by accurate velocity measurements.
     * @param constants The {@link PIDVConstants}.
     */
    public PIDVController(PIDVConstants constants) {
        this.constants = constants;
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
    public PIDVController setTarget(double targetPosition) {
        this.targetPosition = targetPosition;
        return this;
    }

    /** Sets the goal */
    public PIDVController setTarget(double targetPosition, double targetSpeed) {
        this.targetPosition = targetPosition;
        this.targetSpeed = targetSpeed;
        return this;
    }

    public double getOutput() {
        if (isContinuous) {
            error = MathUtil.inputModulus(
                targetPosition - positionMeasurement, 
                -(maxRange - minRange) / 2.0, 
                (maxRange - minRange) / 2.0);
        } else {
            error = targetPosition - positionMeasurement;
        }

        integral += error * Constants.DT;

        double derivative = targetSpeed - velocityMeasurement;

        return constants.kP * error
            + constants.kI * integral
            + constants.kD * derivative;
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
        error = 0.0;
        return this;
    }
}
