package frc.robot.lib.io;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.lib.sim.Simulation;

public class ISimTalonFX extends ITalonFX {
	protected TalonFXSimState simState;
	protected Simulation simulation;
	protected Notifier simNotifier;

	public ISimTalonFX(TalonFX talonFx, Simulation simulation, String key) {
		super(talonFx, key);
		simState = talonFx.getSimState();
		this.simulation = simulation;
		simNotifier =
				new Notifier(
						() -> {
							updateSim(0.01);
						});
		simNotifier.setName(key);
		simNotifier.startPeriodic(0.01);
	}

	public void updateSim(double dt) {
		simState.setSupplyVoltage(12);
		double voltage = simState.getMotorVoltage();
		simulation.input(voltage);
		simulation.update(dt);
		simState.setRawRotorPosition(simulation.getPosition() * simulation.getRatio());
		simState.setRotorVelocity(simulation.getVelocity() * simulation.getRatio());
		simState.setRotorAcceleration(simulation.getAcceleration() * simulation.getRatio());
    	refresh();
	}
}
