package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.IMotor;
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

		StructPublisher<Pose3d> publisher =
				NetworkTableInstance.getDefault()
						.getStructTopic("SmartDashboard/Mechanisms/Intake", Pose3d.struct)
						.publish();

		var thread =
				new Notifier(
						() -> {
							publisher.accept(
									new Pose3d(
											new Translation3d(-0.349, -0.293225, 0),
											new Rotation3d(Rotations.of(0), io.getPosition(), Rotations.of(0))));
						});

		thread.startPeriodic(1.0 / 60.0);
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
