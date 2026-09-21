package frc.robot.lib.subsystem;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.Set;

public abstract class LoggedSubsystem extends SubsystemBase {
	private static final Set<LoggedSubsystem> subsystems = new HashSet<>();

	public LoggedSubsystem() {
		super();
		subsystems.add(this);
	}

	public static void logAll() {
		for (LoggedSubsystem s : subsystems) {
			s.log();
		}
	}

	protected void log() {
		DogLog.log(
				"Commands/" + getName(),
				getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
		;
	}
}
