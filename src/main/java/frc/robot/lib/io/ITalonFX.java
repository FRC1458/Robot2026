package frc.robot.lib.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.lib.util.CtreUtil;

public class ITalonFX implements IMotor {
	public final String key;
	protected String positionKey;
	protected String velocityKey;
	protected String voltageKey;
	protected String statorCurrentKey;
	protected String supplyCurrentKey;
	protected String temperatureKey;

	protected TalonFX talonFx;

	protected RunMode mode = RunMode.VOLTAGE;
	protected NeutralOut neutralOut = new NeutralOut();
	protected DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
	protected VoltageOut voltageOut = new VoltageOut(0.0);
	protected PositionVoltage positionVoltage = new PositionVoltage(0.0);
	protected VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
	protected PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0);
	protected VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0);
	protected MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
	protected MotionMagicVelocityVoltage motionMagicVelocityVoltage =
			new MotionMagicVelocityVoltage(0.0);
	protected MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC =
			new MotionMagicTorqueCurrentFOC(0.0);
	protected MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityTorqueCurrentFOC =
			new MotionMagicVelocityTorqueCurrentFOC(0.0);
	protected MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0.0);
	protected MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC =
			new MotionMagicExpoTorqueCurrentFOC(0.0);

	protected TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

	protected Voltage positionVoltageFeedforward = Volts.of(0.0);
	protected Voltage velocityVoltageFeedforward = Volts.of(0.0);
	protected Current positionCurrentFeedforward = Amps.of(0.0);
	protected Current velocityCurrentFeedforward = Amps.of(0.0);

	protected StatusSignal<Angle> positionSignal;
	protected StatusSignal<AngularVelocity> velocitySignal;
	protected StatusSignal<Voltage> voltageSignal;
	protected StatusSignal<Current> statorCurrentSignal;
	protected StatusSignal<Current> supplyCurrentSignal;
	protected StatusSignal<Temperature> temperatureSignal;

	public ITalonFX(TalonFX talonFx, String key) {
		this.key = key;
		positionKey = key + "/Position";
		velocityKey = key + "/Velocity";
		voltageKey = key + "/Voltage";
		statorCurrentKey = key + "/StatorCurrent";
		supplyCurrentKey = key + "/SupplyCurrent";
		temperatureKey = key + "/Temperature";

		this.talonFx = talonFx;
		positionSignal = talonFx.getPosition();
		velocitySignal = talonFx.getVelocity();
		voltageSignal = talonFx.getMotorVoltage();
		statorCurrentSignal = talonFx.getStatorCurrent();
		supplyCurrentSignal = talonFx.getSupplyCurrent();
		temperatureSignal = talonFx.getDeviceTemp();

		try (Notifier thread = new Notifier(() -> refresh())) {
			thread.startPeriodic(0.02);
		}
	}

	@Override
	public void setRunMode(RunMode mode) {
		this.mode = mode;
	}

	@Override
	public void setNeutral() {
		talonFx.setControl(neutralOut);
	}

	@Override
	public void setDutyCycle(double dutyCycle) {
		dutyCycleOut.withOutput(dutyCycle);
		talonFx.setControl(dutyCycleOut);
	}

	@Override
	public void setVoltage(Voltage voltage) {
		voltageOut.withOutput(voltage);
		talonFx.setControl(voltageOut);
	}

	@Override
	public void setPosition(Angle position) {
		switch (mode) {
			case VOLTAGE:
				positionVoltage.withPosition(position).withFeedForward(positionVoltageFeedforward);
				talonFx.setControl(positionVoltage);
				break;
			case VOLTAGE_TRAPEZOIDAL:
				motionMagicVoltage.withPosition(position).withFeedForward(positionVoltageFeedforward);
				talonFx.setControl(motionMagicVoltage);
				break;
			case VOLTAGE_EXPONENTIAL:
				motionMagicExpoVoltage.withPosition(position).withFeedForward(positionVoltageFeedforward);
				talonFx.setControl(motionMagicExpoVoltage);
				break;
			case CURRENT:
				positionTorqueCurrentFOC.withPosition(position).withFeedForward(positionCurrentFeedforward);
				talonFx.setControl(positionTorqueCurrentFOC);
				break;
			case CURRENT_TRAPEZOIDAL:
				motionMagicTorqueCurrentFOC
						.withPosition(position)
						.withFeedForward(positionCurrentFeedforward);
				talonFx.setControl(motionMagicTorqueCurrentFOC);
				break;
			case CURRENT_EXPONENTIAL:
				motionMagicExpoTorqueCurrentFOC
						.withPosition(position)
						.withFeedForward(positionCurrentFeedforward);
				talonFx.setControl(motionMagicExpoTorqueCurrentFOC);
		}
	}

	@Override
	public void setVelocity(AngularVelocity velocity) {
		switch (mode) {
			case VOLTAGE:
				velocityVoltage.withVelocity(velocity).withFeedForward(velocityVoltageFeedforward);
				talonFx.setControl(velocityVoltage);
				break;
			case VOLTAGE_TRAPEZOIDAL:
				motionMagicVelocityVoltage
						.withVelocity(velocity)
						.withFeedForward(velocityVoltageFeedforward);
				talonFx.setControl(motionMagicVelocityVoltage);
				break;
			case CURRENT:
				velocityTorqueCurrentFOC.withVelocity(velocity).withFeedForward(velocityCurrentFeedforward);
				talonFx.setControl(velocityTorqueCurrentFOC);
				break;
			case CURRENT_TRAPEZOIDAL:
				motionMagicVelocityTorqueCurrentFOC
						.withVelocity(velocity)
						.withFeedForward(velocityCurrentFeedforward);
				talonFx.setControl(motionMagicVelocityTorqueCurrentFOC);
				break;
			default:
				System.err.println("Velocity control is not supported for Motion Magic Expo");
		}
	}

	@Override
	public void setCurrent(Current current) {
		torqueCurrentFOC.withOutput(current);
		talonFx.setControl(torqueCurrentFOC);
	}

	@Override
	public void addPositionFeedforward(Voltage positionFeedforward) {
		this.positionVoltageFeedforward = positionFeedforward;
	}

	@Override
	public void addVelocityFeedforward(Voltage velocityFeedforward) {
		this.velocityVoltageFeedforward = velocityFeedforward;
	}

	@Override
	public void addPositionFeedforward(Current positionFeedforward) {
		this.positionCurrentFeedforward = positionFeedforward;
	}

	@Override
	public void addVelocityFeedforward(Current velocityFeedforward) {
		this.velocityCurrentFeedforward = velocityFeedforward;
	}

	@Override
	public Angle getPosition() {
		return positionSignal.getValue();
	}

	@Override
	public AngularVelocity getVelocity() {
		return velocitySignal.getValue();
	}

	public void refresh() {
		BaseStatusSignal.refreshAll(
				positionSignal,
				velocitySignal,
				voltageSignal,
				statorCurrentSignal,
				supplyCurrentSignal,
				temperatureSignal);
	}

	@Override
	public void log() {
		DogLog.log(positionKey, positionSignal.getValue());
		DogLog.log(velocityKey, velocitySignal.getValue());
		DogLog.log(voltageKey, voltageSignal.getValue());
		DogLog.log(statorCurrentKey, statorCurrentSignal.getValue());
		DogLog.log(supplyCurrentKey, supplyCurrentSignal.getValue());
		DogLog.log(temperatureKey, temperatureSignal.getValue());
	}

	public void setSlot(int slot) {
		positionVoltage.withSlot(slot);
		velocityVoltage.withSlot(slot);
		positionTorqueCurrentFOC.withSlot(slot);
		velocityTorqueCurrentFOC.withSlot(slot);
		motionMagicVoltage.withSlot(slot);
		motionMagicVelocityVoltage.withSlot(slot);
		motionMagicTorqueCurrentFOC.withSlot(slot);
		motionMagicVelocityTorqueCurrentFOC.withSlot(slot);
		motionMagicExpoVoltage.withSlot(slot);
		motionMagicExpoTorqueCurrentFOC.withSlot(slot);
	}

	public void configure(TalonFXConfiguration config) {
		CtreUtil.applyConfiguration(talonFx, config);
	}

	public TalonFX getMotor() {
		return talonFx;
	}
}
