package frc.robot.lib.sim;

public interface Simulation {
	double getRatio();

	void input(double voltage);

	void update(double dt);

	double getPosition();

	double getVelocity();

	double getAcceleration();
}
