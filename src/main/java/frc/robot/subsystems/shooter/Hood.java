package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import dev.doglog.internal.tunable.Tunable;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ISimTalonFX;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.sim.SingleJointedArmSimulation;
import frc.robot.lib.subsystem.HomingMotorSubsystem;
import java.util.function.DoubleSupplier;

// basically all code taken from IntakePivot.java
public class Hood extends HomingMotorSubsystem {
	// configure this
	public Hood() {
		super(
				() -> {
					TalonFX motor = new TalonFX(HOOD_ID);
					if (RobotBase.isReal()) {
						return new ITalonFX(motor, "Shooter/Hood");
					} else {
						return new ISimTalonFX(
								motor,
								new SingleJointedArmSimulation(
										Rotations.of(1),
										Rotations.of(HOOD_GEAR_RATIO),
										HOOD_MOI,
										HOOD_LENGTH,
										HOOD_POS_MIN,
										HOOD_POS_MAX,
										HOOD_POS_MAX,
										DCMotor.getKrakenX60(1),
										0.0,
										0.0),
								"Shooter/Hood");
					}
				});
		((ITalonFX) io).configure(HOOD_CONFIG);
		SmartDashboard.putNumber("hoodpos", 0);
	}

	public Command a(DoubleSupplier distance) {
		return runOnce(() -> io.setRunMode(RunMode.VOLTAGE_TRAPEZOIDAL))
				.andThen(
						run(
								() -> {
									double deg = ANGLE_MAP.get(distance.getAsDouble());
									io.setPosition(Degrees.of(deg));
								}));
		// Angle angle = Degrees.of(ANGLE_MAP.get(distance.getAsDouble()));
		// return runPos(angle, HOOD_EPS, RunMode.VOLTAGE_TRAPEZOIDAL).withTimeout(1);
	}

	public Command zero() {
		return runPos(HOOD_POS_MIN, HOOD_EPS, RunMode.VOLTAGE_TRAPEZOIDAL).withTimeout(1);
	}

	public Command pass() {
		return runPos(HOOD_POS_MAX, HOOD_EPS, RunMode.VOLTAGE_TRAPEZOIDAL).withTimeout(1);
	}

	public Command waitUntilAligned() {
		return Commands.waitUntil(
				() ->
						Rotations.of(
										((ITalonFX) io).getMotor().getClosedLoopError().getValueAsDouble())
								.isNear(Rotations.of(0), HOOD_EPS));
	}

	public Command reset() {
		return runOnce(() -> {
			((ITalonFX) io).getMotor().setPosition(Degrees.of(SmartDashboard.getNumber("hoodpos", 0)));
		});
	}
}
