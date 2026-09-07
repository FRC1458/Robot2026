package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ISimTalonFX;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.sim.SingleJointedArmSimulation;
import frc.robot.lib.subsystem.HomingMotorSubsystem;

public class IntakePivot extends HomingMotorSubsystem {
	public IntakePivot() {
		super(
				() -> {
					TalonFX motor = new TalonFX(PIVOT_ID);
					if (RobotBase.isReal()) {
						return new ITalonFX(motor, "Intake/Pivot");
					} else {
						return new ISimTalonFX(
								motor,
								new SingleJointedArmSimulation(
										Rotations.of(1),
										Rotations.of(PIVOT_GEAR_RATIO),
										MOI,
										INTAKE_LENGTH,
										PIVOT_POS_MIN,
										PIVOT_POS_MAX,
										PIVOT_POS_MAX,
										DCMotor.getKrakenX60(1),
										0.0,
										0.0),
								"Intake/Pivot");
					}
				});
		((ITalonFX) io).configure(PIVOT_CONFIG);
	}

	public Command lower() {
		return runPos(PIVOT_POS_MIN, PIVOT_EPSILON, RunMode.VOLTAGE_TRAPEZOIDAL).withTimeout(1);
	}

	public Command raise() {
		return runPos(PIVOT_POS_UP, PIVOT_EPSILON, RunMode.VOLTAGE_TRAPEZOIDAL).withTimeout(1);
	}

	public Command shake() {
		return Commands.repeatingSequence(
				runOnce(() -> io.setPosition(PIVOT_POS_MID)),
				Commands.waitSeconds(0.3),
				runOnce(() -> io.setPosition(PIVOT_POS_UP)),
				Commands.waitSeconds(0.3));
	}

	public Command calibrateZero() {
		Trigger isHardStop =
				new Trigger(
								() -> {
									return io.getVelocity().abs(RotationsPerSecond) < 1
											&& ((ITalonFX) io).getMotor().getTorqueCurrent().getValue().abs(Amps) > 15;
								})
						.debounce(0.10);

		return run(() -> {
					io.setVoltage(Volts.of(-1.5));
				})
				.until(isHardStop)
				.andThen(
						runOnce(() -> io.setNeutral())
								.withTimeout(1)
								.finallyDo(
										() -> {
											((ITalonFX) io).getMotor().setPosition(Rotations.of(0));
										}));
	}

	public Command stop() {
		return runOnce(() -> io.setNeutral());
	}
}
