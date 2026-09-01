package frc.robot.lib.sim;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorSimulation implements Simulation {
	ElevatorSim sim;
	double ratio;

	public ElevatorSimulation(
			double gearRatio,
			Mass carriageMass,
			Distance sprocketRadius,
			Distance minHeight,
			Distance maxHeight,
			Distance startingHeight,
			DCMotor gearbox,
			double... measurementStdDevs) {
		sim =
				new ElevatorSim(
						gearbox,
						gearRatio,
						carriageMass.in(Kilograms),
						sprocketRadius.in(Meters),
						minHeight.in(Meters),
						maxHeight.in(Meters),
						true,
						startingHeight.in(Meters),
						measurementStdDevs);

		ratio = gearRatio / (2 * Math.PI * sprocketRadius.in(Meters));
	}

	@Override
	public double getRatio() {
		return ratio;
	}

	@Override
	public void input(double voltage) {
		sim.setInput(voltage);
	}

	@Override
	public void update(double dt) {
		sim.update(dt);
	}

	@Override
	public double getPosition() {
		return sim.getPositionMeters();
	}

	@Override
	public double getVelocity() {
		return sim.getVelocityMetersPerSecond();
	}

	@Override
	public double getAcceleration() {
		return 0.0;
	}
}
