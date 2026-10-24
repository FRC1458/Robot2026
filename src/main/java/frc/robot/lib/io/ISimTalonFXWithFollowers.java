package frc.robot.lib.io;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.lib.sim.Simulation;

public class ISimTalonFXWithFollowers extends ITalonFXWithFollowers {
	protected TalonFXSimState[] simStates;
	protected Simulation simulation;
	protected Notifier simNotifier;

	public ISimTalonFXWithFollowers(
			TalonFX talonFx, Simulation simulation, String key, TalonFXFollower... followers) {
		super(talonFx, key, followers);

		this.simulation = simulation;

		simStates = new TalonFXSimState[followers.length + 1];
		simStates[0] = talonFx.getSimState();

		for (int i = 0; i < followers.length; i++) {
			simStates[i + 1] = followers[i].talonFx().getSimState();
		}

		simNotifier =
				new Notifier(
						() -> {
							updateSim(0.01);
						});
		simNotifier.setName(key);
		simNotifier.startPeriodic(0.01);
	}

	public void updateSim(double dt) {
		for (TalonFXSimState simState : simStates) {
			simState.setSupplyVoltage(12);
		}

		double voltage = simStates[0].getMotorVoltage();

		simulation.input(voltage);
		simulation.update(dt);

		for (int i = 0; i < simStates.length; i++) {
			double alignment = 
					i == 0
									|| followers[i - 1].alignment()
											== com.ctre.phoenix6.signals.MotorAlignmentValue.Aligned
							? 1.0
							: -1.0;

			simStates[i].setRawRotorPosition(
					simulation.getPosition() * simulation.getRatio() * alignment);
			simStates[i].setRotorVelocity(simulation.getVelocity() * simulation.getRatio() * alignment);
			simStates[i].setRotorAcceleration(
					simulation.getAcceleration() * simulation.getRatio() * alignment);
		}

		refresh();
	}
}
