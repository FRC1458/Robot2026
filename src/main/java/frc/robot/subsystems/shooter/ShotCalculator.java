package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.houndlib.ShootOnTheFlyCalculator;
import frc.robot.subsystems.shooter.houndlib.ShootOnTheFlyCalculator.InterceptSolution;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.trajectory.RedTrajectory.State.ChassisAccels;
import frc.robot.subsystems.drive.Drive;

// stores current target and actively computes effective target
public class ShotCalculator extends SubsystemBase {
    private final Drive drivetrain;

    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;

    private double currentEffectiveYaw;

    private InterceptSolution currentInterceptSolution;

    private Pose3d targetLocation = new Pose3d();

    private double targetDistance = 0.0;

    private double targetSpeedRps = 8;

    public ShotCalculator() {
        this.drivetrain = Drive.getInstance();
    }

    @Override
    public void periodic() {
        Pose2d drivetrainPose = drivetrain.getPose();

        targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
        targetSpeedRps = ShooterConstants.DISTANCE_TO_SHOT_SPEED.get(targetDistance);

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(ShooterConstants.OFFSET);

        ChassisSpeeds drivetrainSpeeds = drivetrain.getFieldSpeeds();
        ChassisAccels drivetrainAccelerations = ChassisAccels.estimate(drivetrainSpeeds, drivetrain.getPrevFieldSpeeds(), Constants.DT);

        currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(shooterPose, targetLocation,
                drivetrainSpeeds, drivetrainAccelerations, targetSpeedRps,
                5, 0.01);

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        currentEffectiveYaw = currentInterceptSolution.requiredYaw();
    }

    public void setTarget(Pose3d targetLocation, double targetSpeedRps) {
        this.targetLocation = targetLocation;
        this.targetSpeedRps = targetSpeedRps;
    }

    public Pose3d getCurrentEffectiveTargetPose() {
        return currentEffectiveTargetPose;
    }

    public double getCurrentEffectiveYaw() {
        return currentEffectiveYaw;
    }

    public InterceptSolution getInterceptSolution() {
        return currentInterceptSolution;
    }
}
