package frc.robot.auto;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.field.FieldLayout;
import frc.robot.subsystems.coralshooter.CoralShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
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
        return Commands.defer(() -> {
            Pose2d target = Drive.getInstance().getPose()
                .nearest(left ? FieldLayout.ALIGN_POSES_LEFT : FieldLayout.ALIGN_POSES_RIGHT);

            double alignTime = PIDToPoseCommand.estimateTimeToPose(
                Drive.getInstance().getPose(),
                Drive.getInstance().getFieldSpeeds(),
                target);

            double elevatorTime = Elevator.getInstance().estimateTimeToTarget(level.height);

            Command autoAlign = Drive.getInstance().autoAlign(left);

            return Commands.parallel(
                autoAlign,
                Commands.waitSeconds(alignTime - elevatorTime)
                    .andThen(Elevator.getInstance().moveToScoringHeight(level.height))
            ).andThen(
                CoralShooter.getInstance().shoot());
        },
        Set.of(Drive.getInstance(), Elevator.getInstance(), CoralShooter.getInstance()));
    }
}
