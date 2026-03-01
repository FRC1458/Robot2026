package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
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

    private final Drive drive;

    @AutoLogOutput
    private Translation3d currentEffectiveTargetPose = Translation3d.kZero;
    private double currentEffectiveYaw;

    @AutoLogOutput
    private InterceptSolution currentInterceptSolution;
    private Translation3d targetLocation = new Translation3d();
    private double targetDistance = 0.0;
    private double shooterAngle = 75 * Constants.TAU / 360;

    private ChassisSpeeds zero = new ChassisSpeeds();
    private ChassisAccels zero1 = new ChassisAccels();

    private ShotCalculator() {
        this.drive = Drive.getInstance();
        AutoLogOutputManager.addObject(this);
    }

    @Override
    public void periodic() {
        // Pose2d drivePose = drive.getPose();

        // targetDistance = drivePose.getTranslation().getDistance(targetLocation.toTranslation2d());

        // var shooterPose = new Pose3d(drivePose).plus(ShooterConstants.OFFSET).getTranslation();

        
        // ChassisSpeeds driveSpeeds = drive.getFieldSpeeds();
        // ChassisAccels driveAccelerations = ChassisAccels.estimate(driveSpeeds, drive.getPrevFieldSpeeds(), Constants.DT);

        // currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(
        //     shooterPose, 
        //     targetLocation,
        //     zero, 
        //     zero1, 
        //     -shooterAngle,
        //     5, 0.01);

        // currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        // currentEffectiveYaw = currentInterceptSolution.requiredYaw();
    }

    public void setTarget(Translation3d targetLocation) {
        this.targetLocation = targetLocation;
    }

    public Translation3d getCurrentEffectiveTargetPose() {
        return currentEffectiveTargetPose;
    }

    public double getCurrentEffectiveYaw() {
        return currentEffectiveYaw;
    }

    public InterceptSolution getInterceptSolution() {
        return currentInterceptSolution;
    }
}
