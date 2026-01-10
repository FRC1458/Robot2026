package frc.robot.lib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDVConstants;

public class ProfiledPIDVController {
	final ProfiledPIDVConstants constants;

	final PIDVController controller;

	final TrapezoidProfile profile;
	TrapezoidProfile.State goal = new TrapezoidProfile.State();
	TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

	public ProfiledPIDVController(ProfiledPIDVConstants constants) {
		this.constants = constants;
		this.controller = new PIDVController(constants.getPIDVConstants());
		profile = new TrapezoidProfile(constants.constraints);
	}

	/**
	 * Makes the controller continuous, which means that values repeat.
	 */
	public ProfiledPIDVController enableContinuousInput(double minInput, double maxInput) {
		controller.enableContinuousInput(minInput, maxInput);
		return this;
	}

	/** Makes the controller discontinuous */
	public ProfiledPIDVController disableContinuousInput() {
		controller.disableContinuousInput();
		return this;
	}

	public ProfiledPIDVController setTarget(double targetPosition) {
		goal = new TrapezoidProfile.State(targetPosition, 0);
		return this;
	}

	public ProfiledPIDVController setTarget(double targetPosition, double targetSpeed) {
		goal = new TrapezoidProfile.State(targetPosition, targetSpeed);
		return this;
	}

	public ProfiledPIDVController setMeasurement(double position, double speed) {
		controller.setMeasurement(position, speed);
		return this;
	}

	public double getOutput() {
		if (controller.isContinuous) {
			double errorBound = (controller.maxRange - controller.minRange) / 2.0;

			double goalDelta =
				MathUtil.inputModulus(goal.position - controller.positionMeasurement, -errorBound, errorBound);
			double setpointDelta =
				MathUtil.inputModulus(setpoint.position - controller.positionMeasurement, -errorBound, errorBound);

			goal.position = goalDelta + controller.positionMeasurement;
			setpoint.position = setpointDelta + controller.positionMeasurement;
		}

		// Advance profile by one timestep
		setpoint = profile.calculate(Constants.DT, setpoint, goal);

		SmartDashboard.putNumber("Debug/Profile", setpoint.velocity);

		// Use profiled position & velocity as setpoints
		return controller.setTarget(setpoint.position, setpoint.velocity).getOutput();
	}

	public double getError() {
		return MathUtil.inputModulus(
			goal.position - controller.positionMeasurement, 
			controller.minRange, 
			controller.maxRange);
	}

	/** Resets the controller. */
	public ProfiledPIDVController reset(double pos, double vel) {
		controller.reset();
		setpoint = new TrapezoidProfile.State(pos, vel);
		return this;
	}	
	
	public PIDVController getController() {
		return controller;
	}
}
