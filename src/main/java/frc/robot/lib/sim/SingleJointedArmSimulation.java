package frc.robot.lib.sim;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SingleJointedArmSimulation implements Simulation {
	SingleJointedArmSim sim;
	double ratio;

	public SingleJointedArmSimulation(
			Angle output,
			Angle input,
			MomentOfInertia moi,
			Distance cogPivotDistance,
			Angle minHeight,
			Angle maxHeight,
			Angle startingHeight,
			DCMotor gearbox,
			double... measurementStdDevs) {
		double gearRatio = input.in(Rotations) / output.in(Rotations);
		double radiansToRotor = input.in(Rotations) / output.in(Radians);
		sim =
				new SingleJointedArmSim(
						gearbox,
						gearRatio,
						moi.in(KilogramSquareMeters),
						cogPivotDistance.in(Meters),
						minHeight.in(Radians),
						maxHeight.in(Radians),
						true,
						startingHeight.in(Radians),
						measurementStdDevs);
		ratio = radiansToRotor;
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
		return sim.getAngleRads();
	}

	@Override
	public double getVelocity() {
		return sim.getVelocityRadPerSec();
	}

	@Override
	public double getAcceleration() {
		return 0.0;
	}
}
