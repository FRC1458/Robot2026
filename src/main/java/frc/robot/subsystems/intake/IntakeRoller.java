package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.lib.util.Util.InchSqPounds;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ISimTalonFX;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.sim.FlywheelSimulation;
import frc.robot.lib.subsystem.RollerMotorSubsystem;

public class IntakeRoller extends RollerMotorSubsystem {
	public IntakeRoller() {
		super(
				() -> {
					TalonFX motor = new TalonFX(ROLLER_ID);
					if (RobotBase.isReal()) {
						return new ITalonFX(motor, "Intake/Roller");
					} else {
						return new ISimTalonFX(
								motor,
								new FlywheelSimulation(
										Rotations.of(1),
										Rotations.of(2),
										InchSqPounds.of(2.598671),
										DCMotor.getKrakenX44(1),
										0.0),
								"Intake/Roller");
					}
				});

		((ITalonFX) io).configure(ROLLER_CONFIG);

		setDefaultCommand(stop());
	}

	public Command intake() {
		return runVel(ROLLER_SPEED, EPS, RunMode.VOLTAGE).andThen(idle());
	}

	public Command outtake() {
		return runVel(OUTTAKE_SPEED, EPS, RunMode.VOLTAGE).andThen(idle());
	}

	public Command stop() {
		return runVel(RotationsPerSecond.of(0), EPS, RunMode.VOLTAGE);
	}
}
