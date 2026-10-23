package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.lib.util.Util.InchSqPounds;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ISimTalonFXWithFollowers;
import frc.robot.lib.io.ITalonFXWithFollowers;
import frc.robot.lib.io.ITalonFXWithFollowers.TalonFXFollower;
import frc.robot.lib.sim.FlywheelSimulation;
import frc.robot.lib.subsystem.RollerMotorSubsystem;

public class Indexer extends RollerMotorSubsystem {
	public Indexer() {
		super(
				() -> {
					TalonFX left = new TalonFX(L_MOTOR_ID, "CV");
					TalonFX right = new TalonFX(R_MOTOR_ID, "CV");

					if (RobotBase.isReal()) {
						return new ITalonFXWithFollowers(
								left, "Indexer", new TalonFXFollower(right, MotorAlignmentValue.Opposed));
					} else {
						return new ISimTalonFXWithFollowers(
								left,
								new FlywheelSimulation(
										Rotations.of(1),
										Rotations.of(1),
										InchSqPounds.of(5),
										DCMotor.getKrakenX44(1),
										0.0),
								"Indexer",
								new TalonFXFollower(right, MotorAlignmentValue.Opposed));
					}
				});

		((ITalonFXWithFollowers) io).configure(L_CONFIG);

		setDefaultCommand(stop());
	}

	public Command index() {
		return runVel(FEEDER_SPEED, RotationsPerSecond.of(10), RunMode.VOLTAGE).withTimeout(1);
	}

	public Command stop() {
		return runOnce(() -> io.setNeutral());
	}
}
