package frc.robot.lib.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelSimulation implements Simulation {
	FlywheelSim sim;
	double ratio;
	double acc;

	public FlywheelSimulation(
			Angle output,
			Angle input,
			MomentOfInertia moi,
			DCMotor gearbox,
			double... measurementStdDevs) {
		ratio = input.in(Rotations) / output.in(Radians);
		sim =
				new FlywheelSim(
						LinearSystemId.createFlywheelSystem(gearbox, moi.in(KilogramSquareMeters), ratio),
						gearbox,
						measurementStdDevs);
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
		acc += dt * sim.getAngularVelocityRPM();
	}

	@Override
	public double getPosition() {
		return acc;
	}

	@Override
	public double getVelocity() {
		return sim.getAngularVelocityRadPerSec();
	}

	@Override
	public double getAcceleration() {
		return sim.getAngularAccelerationRadPerSecSq();
	}
}
