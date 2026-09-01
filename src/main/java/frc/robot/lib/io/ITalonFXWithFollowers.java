package frc.robot.lib.io;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ITalonFXWithFollowers extends ITalonFX {
	protected record TalonFXFollower(TalonFX talonFx, MotorAlignmentValue alignment) {}

	protected String[] positionKeys;
	protected String[] velocityKeys;
	protected String[] voltageKeys;
	protected String[] statorCurrentKeys;
	protected String[] supplyCurrentKeys;
	protected String[] temperatureKeys;

	protected TalonFX talonFx;
	protected TalonFXFollower[] followers;

	protected RunMode mode = RunMode.VOLTAGE;

	protected StatusSignal<Angle>[] positionSignals;
	protected StatusSignal<AngularVelocity>[] velocitySignals;
	protected StatusSignal<Voltage>[] voltageSignals;
	protected StatusSignal<Current>[] statorCurrentSignals;
	protected StatusSignal<Current>[] supplyCurrentSignals;
	protected StatusSignal<Temperature>[] temperatureSignals;

	@SuppressWarnings("unchecked")
	public ITalonFXWithFollowers(TalonFX talonFx, String key, TalonFXFollower... followers) {
		super(talonFx, key);
		this.followers = followers;

		for (TalonFXFollower follower : followers) {
			follower.talonFx.setControl(new Follower(talonFx.getDeviceID(), follower.alignment));
		}

		TalonFX[] motors = new TalonFX[followers.length + 1];
		motors[0] = talonFx;

		for (int i = 0; i < followers.length; i++) {
			motors[i + 1] = followers[i].talonFx;
		}

		positionSignals = new StatusSignal[motors.length];
		velocitySignals = new StatusSignal[motors.length];

		for (int i = 0; i < motors.length; i++) {
			positionSignals[i] = motors[i].getPosition();
			velocitySignals[i] = motors[i].getVelocity();
		}

		positionSignals = new StatusSignal[motors.length];
		velocitySignals = new StatusSignal[motors.length];
		voltageSignals = new StatusSignal[motors.length];
		statorCurrentSignals = new StatusSignal[motors.length];
		supplyCurrentSignals = new StatusSignal[motors.length];
		temperatureSignals = new StatusSignal[motors.length];

		for (int i = 0; i < motors.length; i++) {
			positionSignals[i] = motors[i].getPosition();
			velocitySignals[i] = motors[i].getVelocity();
			voltageSignals[i] = motors[i].getMotorVoltage();
			statorCurrentSignals[i] = motors[i].getStatorCurrent();
			supplyCurrentSignals[i] = motors[i].getSupplyCurrent();
			temperatureSignals[i] = motors[i].getDeviceTemp();
		}

		positionKeys = new String[motors.length];
		velocityKeys = new String[motors.length];
		voltageKeys = new String[motors.length];
		statorCurrentKeys = new String[motors.length];
		supplyCurrentKeys = new String[motors.length];
		temperatureKeys = new String[motors.length];

		for (int i = 0; i < motors.length; i++) {
			positionKeys[i] = key + "/Motor" + i + "/Position";
			velocityKeys[i] = key + "/Motor" + i + "/Velocity";
			voltageKeys[i] = key + "/Motor" + i + "/Voltage";
			statorCurrentKeys[i] = key + "/Motor" + i + "/StatorCurrent";
			supplyCurrentKeys[i] = key + "/Motor" + i + "/SupplyCurrent";
			temperatureKeys[i] = key + "/Motor" + i + "/Temperature";
		}
	}

	@Override
	public Angle getPosition() {
		double position = 0.0;

		for (StatusSignal<Angle> signal : positionSignals) {
			position += signal.getValue().in(Rotations);
		}

		position /= positionSignals.length;

		return Rotations.of(position);
	}

	@Override
	public AngularVelocity getVelocity() {
		double velocity = 0.0;

		for (StatusSignal<AngularVelocity> signal : velocitySignals) {
			velocity += signal.getValue().in(RotationsPerSecond);
		}

		velocity /= velocitySignals.length;

		return RotationsPerSecond.of(velocity);
	}

	@Override
	public void refresh() {
		for (int i = 0; i < positionSignals.length; i++) {
			BaseStatusSignal.refreshAll(
					positionSignals[i],
					velocitySignals[i],
					voltageSignals[i],
					statorCurrentSignals[i],
					supplyCurrentSignals[i],
					temperatureSignals[i]);
		}
	}

	@Override
	public void log() {
		for (int i = 0; i < positionSignals.length; i++) {
			DogLog.log(positionKeys[i], positionSignals[i].getValue());
			DogLog.log(velocityKeys[i], velocitySignals[i].getValue());
			DogLog.log(voltageKeys[i], voltageSignals[i].getValue());
			DogLog.log(statorCurrentKeys[i], statorCurrentSignals[i].getValue());
			DogLog.log(supplyCurrentKeys[i], supplyCurrentSignals[i].getValue());
			DogLog.log(temperatureKeys[i], temperatureSignals[i].getValue());
		}

		DogLog.log(positionKey, getPosition());
		DogLog.log(velocityKey, getVelocity());
	}
}
