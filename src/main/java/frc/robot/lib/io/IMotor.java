package frc.robot.lib.io;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IMotor {
	public static enum RunMode {
		VOLTAGE,
		VOLTAGE_TRAPEZOIDAL,
		VOLTAGE_EXPONENTIAL,

		CURRENT,
		CURRENT_TRAPEZOIDAL,
		CURRENT_EXPONENTIAL
	}

	void setRunMode(RunMode mode);

	void setNeutral();

	void setDutyCycle(double dutyCycle);

	void setVoltage(Voltage voltage);

	void setPosition(Angle position);

	void setVelocity(AngularVelocity velocity);

	void setCurrent(Current current);

	void addPositionFeedforward(Current feedforward);

	void addVelocityFeedforward(Current feedforward);

	void addPositionFeedforward(Voltage feedforward);

	void addVelocityFeedforward(Voltage feedforward);

	Angle getPosition();

	AngularVelocity getVelocity();

	void log();
}
