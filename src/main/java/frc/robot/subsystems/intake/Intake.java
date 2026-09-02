package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public IntakePivot pivot;
    public IntakeRoller roller;

    public Intake(IntakePivot pivot, IntakeRoller roller) {
        this.pivot = pivot;
        this.roller = roller;
    }

    public Command intake() {
        return Commands.parallel(
            pivot.lower(),
            roller.intake()
        );
    }

    public Command raise() {
        return Commands.parallel(
            pivot.raise(),
            roller.intake()
        );
    }

    public Command agitate() {
        return Commands.parallel(
            pivot.shake(),
            roller.intake()
        );
    }

    public Command outtake() {
        return Commands.parallel(
            pivot.lower(),
            roller.outtake()
        );
    }

    public Command stop() {
        return Commands.parallel(
            pivot.stop(),
            roller.stop()
        );
    }
}
