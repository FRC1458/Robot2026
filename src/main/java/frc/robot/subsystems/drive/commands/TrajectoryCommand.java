package frc.robot.subsystems.drive.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.lib.control.ControlConstants.PIDFConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.control.PIDVController;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.trajectory.RedTrajectory;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.Drive;

/**
 * Command that follows a trajectory
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
            DriveConstants.TRANSLATION_CONSTANTS, 
            DriveConstants.ROTATION_CONSTANTS, 
            DriveConstants.ACCELERATION_CONSTANT);
    }
    
    /**
     * A drive controller that works with 2 {@link PIDVController}s for translation and one {@link ProfiledPIDVController} for rotation.
     * @param translationConstants The {@link PIDFConstants} for the translation of the robot.
     * @param rotationConstants The {@link ProfiledPIDFConstants} for the rotation of the robot.
     * @param accelConstant The acceleration feedforwards (useful for traversing sharp turns on a trajectory).
     */
    public TrajectoryCommand(Drive drive, RedTrajectory trajectory, PIDFConstants translationConstants, ProfiledPIDFConstants rotationConstants, double accelConstant) {
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
        setName("Trajectory " + trajectory.name);
    }

    @Override
    public void initialize() {
        timer.start();
        drive.setSwerveRequest(request);
    }

    @Override
    public void execute() {
        setRobotState(
            drive.getPose(), drive.getFieldSpeeds());
        // updates the request
        request.withSpeeds(calculateSpeeds());
    }

    public void setRobotState(Pose2d pose, ChassisSpeeds speeds) {
        this.currentPose = pose;
        this.currentSpeeds = speeds;
    }

    public ChassisSpeeds calculateSpeeds() {
        if (trajectory == null || currentPose == null || currentSpeeds == null || trajectory.isDone()) {
            return new ChassisSpeeds();
        }

        targetState = trajectory.advanceTo(timer.get());

        double vxFF = targetState.speeds.vxMetersPerSecond;
        double vyFF = targetState.speeds.vyMetersPerSecond;

        double xAccelFF = MathUtil.applyDeadband(
            targetState.accels.ax,    
            DriveConstants.MAX_ACCEL * 0.5);
        double yAccelFF = MathUtil.applyDeadband(
            targetState.accels.ay,
            DriveConstants.MAX_ACCEL * 0.5);

        double angularAccel = MathUtil.applyDeadband(
            targetState.accels.alpha,
            DriveConstants.MAX_ROTATION_ACCEL * 0.5);
        xAccelFF += -angularAccel * targetState.pose.getRotation().getSin();
        yAccelFF += angularAccel * targetState.pose.getRotation().getCos();

        xController.setTarget(targetState.pose.getX());
        yController.setTarget(targetState.pose.getY());

        xController.setFeedforward(vxFF);
        yController.setFeedforward(vyFF);

        xController.setInput(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
        yController.setInput(currentPose.getY(), currentSpeeds.vyMetersPerSecond);

        double vx = xController.getOutput();
        double vy = yController.getOutput();

        thetaController.setTarget(targetState.pose.getRotation().getRadians());
        thetaController.setFeedforward(targetState.speeds.omegaRadiansPerSecond);
        thetaController.setInput(
            currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);

        double rotation = thetaController.getOutput();

        return new ChassisSpeeds(
            vx + xAccelFF * accelConstant,
            vy + yAccelFF * accelConstant,
            rotation);
    }

    @Override
    public boolean isFinished() {
        if (trajectory == null) {
            return true;
        }
        if (trajectory.isDone()) {
            System.out.println("Done with trajectory, error: " + Math.hypot(xController.error, yController.error));
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setSwerveRequest(new SwerveRequest.FieldCentric());
    }

    public RedTrajectory getTrajectory() {
        return trajectory;
    }
}
