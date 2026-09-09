package frc.robot.lib.trajectory;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class LocalADStarWrapper {
    private LocalADStar a;
    private Rotation2d initialRotation;
    private Rotation2d finalRotation;
    public LocalADStarWrapper() {
        a = new LocalADStar();
    }

    public void setInitialPose(Pose2d pose) {
        a.setStartPosition(pose.getTranslation());
        initialRotation = pose.getRotation();
    }

    public void setFinalPose(Pose2d pose) {
        a.setGoalPosition(pose.getTranslation());
        finalRotation = pose.getRotation();
    }

    public boolean hasPath() {
        return a.isNewPathAvailable();
    }

    public RedTrajectory getPath() {
        return new RedTrajectory(
            a.getCurrentPath(
                Constants.Pathplanner.GLOBAL_CONSTRAINTS, 
                new GoalEndState(0, finalRotation)
            ).generateTrajectory(
                Drive.getInstance().getFieldSpeeds(), initialRotation, 
                Constants.Pathplanner.config),  
                false
            );
    }
}
