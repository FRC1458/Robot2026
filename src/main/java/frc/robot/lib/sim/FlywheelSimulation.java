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
		var gearRatio = input.in(Rotations) / output.in(Rotations);
		sim =
				new FlywheelSim(
						LinearSystemId.createFlywheelSystem(gearbox, moi.in(KilogramSquareMeters), gearRatio),
						gearbox,
						measurementStdDevs);
		ratio = input.in(Rotations) / output.in(Radians);
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
		acc += dt * sim.getAngularVelocityRadPerSec();
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
