package frc.robot.subsystems.drive.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.control.ControlConstants.*;
import frc.robot.lib.control.*;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.Drive;

/**
 * Command that follows a trajectory
 * TODO: fix the profiled pid controller
 */
public class TrajectoryCommand extends Command {
    public final Drive drive;

    private final SwerveRequest.ApplyFieldSpeeds request = 
        new SwerveRequest.ApplyFieldSpeeds();

    private final PIDVController xController;
    private final PIDVController yController;
    private final ProfiledPIDVController thetaController;
    private double accelConstant;

    private final RedTrajectory trajectory;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;
    private RedTrajectory.State targetState = new RedTrajectory.State();

    private Timer timer = null;

    public TrajectoryCommand(RedTrajectory trajectory) {
        this(
            Drive.getInstance(), 
            trajectory, 
            TRANSLATION_CONSTANTS, 
            PROFILED_ROTATION_CONSTANTS, 
            ACCELERATION_CONSTANT);
    }
    
    /**
     * A drive controller that works with 2 {@link PIDVController}s for translation and one {@link PIDVController} for rotation.
     * @param translationConstants The {@link PIDVConstants} for the translation of the robot.
     * @param rotationConstants The {@link PIDVConstants} for the rotation of the robot.
     * @param accelConstant The acceleration feedforwards (useful for traversing sharp turns on a trajectory).
     */
    public TrajectoryCommand(Drive drive, RedTrajectory trajectory, PIDVConstants translationConstants, ProfiledPIDVConstants rotationConstants, double accelConstant) {
        this.drive = drive;
        this.trajectory = trajectory;
        xController = new PIDVController(translationConstants);
        yController = new PIDVController(translationConstants);
        thetaController = new ProfiledPIDVController(rotationConstants);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.accelConstant = accelConstant;

        timer = new Timer();
        addRequirements(drive);
        TelemetryManager.getInstance()
            .addStructPublisher("Debug/TrajectoryCommand", 
                Pose3d.struct, () -> new Pose3d(
                    targetState.pose));
        
        tracker = new RMSTracker();
        setName(trajectory.name + " :Trajectory");
    }

    @Override
    public void initialize() {
        timer.start(); // actually starts the timer
        drive.setSwerveRequest(request);
    }

    @Override
    public void execute() {
        setRobotState(
            drive.getPose(), drive.getFieldSpeeds());

        // Advances along the trajectory
        targetState = trajectory.advanceTo(timer.get());
        
        tracker.updateRmsError(timer, xController, yController, thetaController.getController());
        // updates the request
        request.withSpeeds(calculateSpeeds());
    }

    public void setRobotState(Pose2d pose, ChassisSpeeds speeds) {
        this.currentPose = pose;
        this.currentSpeeds = speeds;
    }

    public ChassisSpeeds calculateSpeeds() {
        if (trajectory == null || currentPose == null || currentSpeeds == null || trajectory.isDone()) {
            return new ChassisSpeeds(); // Safety
        }

        // gets the feedforwards for translation
        double vxFF = targetState.speeds.vxMetersPerSecond;
        double vyFF = targetState.speeds.vyMetersPerSecond;

        double xAccelFF = MathUtil.applyDeadband(
            targetState.accels.ax,    
            MAX_ACCEL * 0.5);
        double yAccelFF = MathUtil.applyDeadband(
            targetState.accels.ay,
            MAX_ACCEL * 0.5);
        double angularAccel = MathUtil.applyDeadband(
            targetState.accels.alpha,
            MAX_ROTATION_ACCEL * 0.5); // what did i even want to accomplish from this
            
        // acceleration feedforwards
        xAccelFF += -angularAccel * targetState.pose.getRotation().getSin();
        yAccelFF += angularAccel * targetState.pose.getRotation().getCos();

        double vx = xController.setTarget(targetState.pose.getX(), targetState.speeds.vxMetersPerSecond) // Target setting
            .setMeasurement(currentPose.getX(), currentSpeeds.vxMetersPerSecond) // Measurement setting
            .getOutput(); // Output getting

        // same thing but for vy and rotation instead
        double vy = yController.setTarget(targetState.pose.getY(), targetState.speeds.vyMetersPerSecond)
            .setMeasurement(currentPose.getY(), currentSpeeds.vyMetersPerSecond)
            .getOutput();

        double rotation = thetaController
            .setTarget(
                targetState.pose.getRotation().getRadians(), 
                targetState.speeds.omegaRadiansPerSecond
            ).setMeasurement(
                currentPose.getRotation().getRadians(), 
                currentSpeeds.omegaRadiansPerSecond)
            .getOutput();

        return new ChassisSpeeds(
            vx + vxFF + xAccelFF * accelConstant,
            vy + vyFF + yAccelFF * accelConstant,
            rotation + targetState.speeds.omegaRadiansPerSecond); // Finally
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null) {
            return true; // safety
        }

        if (trajectory.isDone()) {
            System.out.printf(
                "Done with trajectory, error: %.5f m, int error translation: %.5f m*s, rot: %.5f deg*s\n", 
                Math.hypot(xController.getError(), yController.getError()),
                tracker.getTranslationRmsError(),
                tracker.getRotationRmsError() / Math.PI * 180);
            return true; // We are done guys
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // swap out request at end
        drive.setSwerveRequest(new SwerveRequest.FieldCentric());
    }

    /** Helper for getting trajectory from this object */
    public RedTrajectory getTrajectory() {
        return trajectory;
    }


    private RMSTracker tracker;
    /** Helper RMS error tracker to tune PID constants */
    private static class RMSTracker {
        private double translationErrorIntegralSq = 0.0;
        private double rotationErrorIntegralSq = 0.0;
        private double totalTime = 0.0;

        private double lastTimestamp = 0.0;

        private void updateRmsError(
            Timer timer, 
            PIDVController xController,
            PIDVController yController,
            PIDVController rotationController
        ) {
            double now = timer.get();
            double dt = now - lastTimestamp;
            lastTimestamp = now;
            if (dt <= 0.0) {
                return;
            }
            double translationError =
                Math.hypot(
                    xController.getError(),
                    yController.getError());
            translationErrorIntegralSq += translationError * translationError * dt;
            double rotationError =
                rotationController.getError();
            rotationErrorIntegralSq += rotationError * rotationError * dt;
            totalTime += dt;
        }

        public double getTranslationRmsError() {
            if (totalTime <= 0.0) return 0.0;
            return Math.sqrt(translationErrorIntegralSq / totalTime);
        }
        
        public double getRotationRmsError() {
            if (totalTime <= 0.0) return 0.0;
            return Math.sqrt(rotationErrorIntegralSq / totalTime);
        }
    } 
}
