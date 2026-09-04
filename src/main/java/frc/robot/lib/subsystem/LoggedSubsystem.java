package frc.robot.lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.Set;

import dev.doglog.DogLog;

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

	void log() {
		DogLog.log("Commands/" + getName(), getCurrentCommand() != null ? getCurrentCommand().getName() : "None");;
	}
}
