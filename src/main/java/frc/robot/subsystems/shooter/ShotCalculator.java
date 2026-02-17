package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.houndlib.ShootOnTheFlyCalculator;
import frc.robot.lib.houndlib.ShootOnTheFlyCalculator.InterceptSolution;
import frc.robot.lib.trajectory.RedTrajectory.State.ChassisAccels;
import frc.robot.subsystems.drive.Drive;

// stores current target and actively computes effective target
public class ShotCalculator extends SubsystemBase {
    private static ShotCalculator calcInstance;
   
    public static ShotCalculator getInstance() {
		if (calcInstance == null) {
            calcInstance = new ShotCalculator();
		}
		return calcInstance;
	}

    private final Drive drivetrain;

    @AutoLogOutput
    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;

    private double currentEffectiveYaw;

    @AutoLogOutput
    private InterceptSolution currentInterceptSolution;

    private Pose3d targetLocation = new Pose3d();

    private double targetDistance = 0.0;

    private double shooterAngle = 75 * Constants.TAU / 360;

    private ShotCalculator() {
        this.drivetrain = Drive.getInstance();
        AutoLogOutputManager.addObject(this);
    }

    @Override
    public void periodic() {
        Pose2d drivetrainPose = drivetrain.getPose();

        targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(ShooterConstants.OFFSET);

        ChassisSpeeds drivetrainSpeeds = drivetrain.getFieldSpeeds();
        ChassisAccels drivetrainAccelerations = ChassisAccels.estimate(drivetrainSpeeds, drivetrain.getPrevFieldSpeeds(), Constants.DT);

        currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(
            shooterPose, 
            targetLocation,
            drivetrainSpeeds, 
            new ChassisAccels(), 
            -shooterAngle,
            5, 0.01);

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        currentEffectiveYaw = currentInterceptSolution.requiredYaw();
    }

    public void setTarget(Pose3d targetLocation) {
        this.targetLocation = targetLocation;
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
