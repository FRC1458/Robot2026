package frc.robot.subsystems.Shooter.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class Reverse extends Command{
    private final ShooterSubsystem shooter;

    public Reverse(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void execute() {
        shooter.reverse();
    }

    public void end(boolean interrupted) {
        shooter.stop();
    }
}
