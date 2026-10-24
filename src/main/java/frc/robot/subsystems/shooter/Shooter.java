package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.subsystem.LoggedSubsystem;
import java.util.function.DoubleSupplier;

public class Shooter extends LoggedSubsystem {
	public Flywheel flywheel;
	public Hood hood;

	public Shooter(Flywheel bl, Hood hood) {
		super();
		this.flywheel = bl;
		this.hood = hood;
		setDefaultCommand(stopAll());
		SmartDashboard.putNumber("hood", 30);
		SmartDashboard.putNumber("flywheel", 50);
	}

	public Command shootAll(DoubleSupplier distance) {
		return Commands.parallel(hood.a(distance), flywheel.shoot(distance)).andThen(idle());
	}

	public Command pass() {
		return Commands.parallel(flywheel.pass(), hood.pass()).andThen(idle());
	}

	public Command cal() {
		return Commands.parallel(
				flywheel.cal(() -> SmartDashboard.getNumber("flywheel", 0)), 
				hood.cal(() -> SmartDashboard.getNumber("hood", 0)))
			.andThen(idle());
	}

	public Command stopAll() {
		return Commands.parallel(flywheel.stop(), hood.zero()).andThen(idle());
	}

	public Command waitForAll() {
		return Commands.parallel(
						flywheel.waitUntilAtSpeed().asProxy(), hood.waitUntilAligned().asProxy())
				.withTimeout(2);
	}

	public AngularVelocity getVelocity() {
		return flywheel.getIo().getVelocity();
	}

	public Angle getAngle() {
		return hood.getIo().getPosition();
	}

	public Command resetHood() {
		return hood.reset();
	}
}
