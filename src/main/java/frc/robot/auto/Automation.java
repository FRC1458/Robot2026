package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralshooter.CoralShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Automation {
    public static enum Levels {
        L1(ElevatorConstants.Heights.L1),
        L2(ElevatorConstants.Heights.L2),
        L3(ElevatorConstants.Heights.L3),
        L4(ElevatorConstants.Heights.L4);
        private ElevatorConstants.Heights height;
        private Levels(ElevatorConstants.Heights height) {
            this.height = height;
        }
    }

    public static Command score(boolean left, Levels level) {
        return Commands.parallel(
            Drive.getInstance().autoAlign(left),
            Commands.waitSeconds(0.5).andThen(Elevator.getInstance().moveToScoringHeight(level.height))
        ).andThen(
            CoralShooter.getInstance().shoot());
    }
}
