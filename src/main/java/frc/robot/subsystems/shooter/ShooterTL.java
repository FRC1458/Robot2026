package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ISimTalonFX;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.sim.FlywheelSimulation;
import frc.robot.lib.subsystem.RollerMotorSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterTL extends RollerMotorSubsystem {
	public ShooterTL() {
		super(
				() -> {
					TalonFX motor = new TalonFX(TL_ID);
					if (RobotBase.isReal()) {
						return new ITalonFX(motor, "Shooter/TL");
					} else {
						return new ISimTalonFX(
								motor,
								new FlywheelSimulation(
										Rotations.of(1), Rotations.of(1), MOI, DCMotor.getKrakenX60(1), 0.0),
								"Shooter/TL");
					}
				});

		((ITalonFX) io).configure(TL_CONFIG);

		setDefaultCommand(stop());
	}

	public Command shoot(DoubleSupplier distance) {
		return runOnce(() -> io.setRunMode(RunMode.VOLTAGE))
				.andThen(
						run(
								() -> {
									double rps = ShooterConstants.RPS;
									io.setVelocity(RotationsPerSecond.of(rps));
								}));
	}

	public Command pass() {
		return runVel(PASSING_SPEED, EPS, RunMode.VOLTAGE).withTimeout(1);
	}

	public Command waitUntilAtSpeed(AngularVelocity eps) {
		return Commands.waitUntil(
				() ->
						RotationsPerSecond.of(
										((ITalonFX) io).getMotor().getClosedLoopError().getValueAsDouble())
								.isNear(RotationsPerSecond.of(0), eps));
	}

	public Command waitUntilAtSpeed() {
		return waitUntilAtSpeed(EPS);
	}

	public Command stop() {
		return runOnce(() -> io.setNeutral());
	}
}
