package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.lib.util.Util.InchSqPounds;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ISimTalonFX;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.sim.FlywheelSimulation;
import frc.robot.lib.subsystem.RollerMotorSubsystem;

public class LeftIndexer extends RollerMotorSubsystem {
	public LeftIndexer() {
		super(
				() -> {
					TalonFX motor = new TalonFX(L_MOTOR_ID);

					if (RobotBase.isReal()) {
						return new ITalonFX(motor, "Indexer/Left");
					} else {
						return new ISimTalonFX(
								motor,
								new FlywheelSimulation(
										Rotations.of(1),
										Rotations.of(1),
										InchSqPounds.of(5),
										DCMotor.getKrakenX44(1),
										0.0),
								"Indexer/Left");
					}
				});

		((ITalonFX) io).configure(L_CONFIG);

		setDefaultCommand(stop());
	}

	public Command index() {
		return runVel(FEEDER_SPEED, RotationsPerSecond.of(10), RunMode.VOLTAGE).andThen(idle());
	}

	public Command stop() {
		return runVel(RotationsPerSecond.of(0), RotationsPerSecond.of(10), RunMode.VOLTAGE);
	}
}
