package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.subsystem.LoggedSubsystem;

import java.util.function.DoubleSupplier;

public class Shooter extends LoggedSubsystem {
	public ShooterBL bl;
	public ShooterBR br;
	public ShooterTL tl;
	public ShooterTR tr;

	public Shooter(ShooterBL bl, ShooterBR br, ShooterTL tl, ShooterTR tr) {
		super();
		this.bl = bl;
		this.br = br;
		this.tl = tl;
		this.tr = tr;
	}

	public Command shootAll(DoubleSupplier distance) {
		return Commands.parallel(
				bl.shoot(distance), br.shoot(distance), tl.shoot(distance), tr.shoot(distance));
	}

	public Command pass() {
		return Commands.parallel(bl.pass(), br.pass(), tl.pass(), tr.pass());
	}

	public Command stopAll() {
		return Commands.parallel(bl.stop(), br.stop(), tl.stop(), tr.stop());
	}
}
