package frc.robot.lib.subsystem;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    
    abstract void log();
}
