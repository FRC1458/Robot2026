package frc.robot.subsystems.Shooter.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class Spin extends Command{
    private final ShooterSubsystem shooter;

    public Spin(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    public void execute() {
        shooter.spin();
    }

    public void end(boolean interrupted) {
        shooter.stop();
    }
}
