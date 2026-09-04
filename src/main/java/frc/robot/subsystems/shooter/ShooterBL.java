package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ISimTalonFX;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.sim.FlywheelSimulation;
import frc.robot.lib.subsystem.RollerMotorSubsystem;
import java.util.function.DoubleSupplier;

public class ShooterBL extends RollerMotorSubsystem {
	public ShooterBL() {
		super(
				() -> {
					TalonFX motor = new TalonFX(BL_ID);
					if (RobotBase.isReal()) {
						return new ITalonFX(motor, "Shooter/BL");
					} else {
						return new ISimTalonFX(
								motor,
								new FlywheelSimulation(
										Rotations.of(1), Rotations.of(1), MOI, DCMotor.getKrakenX60(1), 0.0),
								"Shooter/BL");
					}
				});

		((ITalonFX) io).configure(BL_CONFIG);

		setDefaultCommand(stop());
	}

	public Command shoot(DoubleSupplier distance) {
		return runOnce(() -> io.setRunMode(RunMode.VOLTAGE))
				.andThen(
						run(
								() -> {
									double rps = VEL_MAP.get(distance.getAsDouble());

									io.setVelocity(RotationsPerSecond.of(rps));
								}));
	}

	public Command pass() {
		return runVel(PASSING_SPEED, RotationsPerSecond.of(10), RunMode.VOLTAGE);
	}

	public Command stop() {
		return runVel(RotationsPerSecond.of(0), RotationsPerSecond.of(10), RunMode.VOLTAGE);
	}
}
