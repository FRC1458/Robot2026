package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ISimTalonFXWithFollowers;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.io.ITalonFXWithFollowers;
import frc.robot.lib.io.ITalonFXWithFollowers.TalonFXFollower;
import frc.robot.lib.sim.FlywheelSimulation;
import frc.robot.lib.subsystem.RollerMotorSubsystem;
import frc.robot.lib.util.CtreUtil;

import java.util.function.DoubleSupplier;

public class Flywheel extends RollerMotorSubsystem {
	public Flywheel() {
		super(
				() -> {
					TalonFX tl = new TalonFX(FLYWHEEL_ID_TL, "rio");
					TalonFX tr = new TalonFX(FLYWHEEL_ID_TR, "rio");
					// DogLog.log("configured1", CtreUtil.applyConfiguration(tr, TR_CONFIG));
					// DogLog.log("configured2", CtreUtil.applyConfiguration(tl, TL_CONFIG));
					CtreUtil.applyConfiguration(tr, TR_CONFIG);
					CtreUtil.applyConfiguration(tl, TL_CONFIG);
					// TalonFX br = new TalonFX(FLYWHEEL_ID_BR);
					if (RobotBase.isReal()) {
						return new ITalonFXWithFollowers(
								tl, "Shooter/Flywheel"
								, 
								new TalonFXFollower(tr, MotorAlignmentValue.Opposed)
								// ,
								// new TalonFXFollower(br, MotorAlignmentValue.Opposed)
								);
					} else {
						return new ISimTalonFXWithFollowers(
								tl,
								new FlywheelSimulation(
										Rotations.of(1), Rotations.of(1), MOI, DCMotor.getKrakenX60(3), 0.0),
								"Shooter/Flywheel"
								,
								new TalonFXFollower(tr, MotorAlignmentValue.Opposed)
								// ,
								// new TalonFXFollower(br, MotorAlignmentValue.Opposed)
								);
					}
				});

		// ((ITalonFXWithFollowers) io).configure(TL_CONFIG);

		setDefaultCommand(stop());
	}

	public Command shoot(DoubleSupplier distance) {
		return runOnce(() -> io.setRunMode(RunMode.CURRENT))
				.andThen(
						run(
								() -> {
									double rps = VEL_MAP.get(distance.getAsDouble());
									io.setVelocity(RotationsPerSecond.of(rps));
								}));
	}

	public Command cal(DoubleSupplier rps) {
		return runOnce(() -> io.setRunMode(RunMode.CURRENT))
				.andThen(
						run(
								() -> {
									double r = rps.getAsDouble();
									io.setVelocity(RotationsPerSecond.of(r));
								}));
	}

	public Command pass() {
		return runVel(PASSING_SPEED, EPS, RunMode.CURRENT).withTimeout(1);
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
