package frc.robot.subsystems.Shooter.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class SpinFast extends Command{
    private final ShooterSubsystem shooter;

    public SpinFast(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void execute() {
        shooter.spinFast();
    }

    public void end(boolean interrupted) {
        shooter.stop();
    }
}
