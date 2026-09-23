package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
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
	}

	public Command a(DoubleSupplier distance) {
		Angle angle = Degrees.of(ANGLE_MAP.get(distance.getAsDouble()));
		return runPos(angle, ShooterConstants.HOOD_EPS, RunMode.VOLTAGE_TRAPEZOIDAL).withTimeout(1);
	}
}
