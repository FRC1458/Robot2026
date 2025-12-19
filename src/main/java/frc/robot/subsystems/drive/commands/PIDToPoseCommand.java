package frc.robot.subsystems.drive.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.control.ControlConstants.PIDFConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.control.PIDVController;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.Drive;

/**
 * A command that moves the drivetrain to a pose.
 */
public class PIDToPoseCommand extends Command {
    public final Drive drive;

    private final SwerveRequest.ApplyFieldSpeeds request = 
        new SwerveRequest.ApplyFieldSpeeds();

    private final PIDVController translationController;
    private final ProfiledPIDVController thetaController;

    private final Pose2d target;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    public PIDToPoseCommand(Pose2d target) {
        this(
            Drive.getInstance(), 
            target,
            TRANSLATION_CONSTANTS, 
            PROFILED_ROTATION_CONSTANTS);
    }
    
    public PIDToPoseCommand(Drive drive, Pose2d target, PIDFConstants translationConstants, ProfiledPIDFConstants rotationConstants) {
        this.drive = drive;
        this.target = target;
        translationController = new PIDVController(translationConstants);
        thetaController = new ProfiledPIDVController(rotationConstants);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
        setName("(" + target.getX() + ", " + target.getY() + ", " 
            + target.getRotation().getDegrees() + " deg)" + " :PID to pose");
    }

    @Override
    public void initialize() {
        drive.setSwerveRequest(request);
    }

    @Override
    public void execute() {
        setRobotState(
            drive.getPose(), drive.getFieldSpeeds());
        // updates the request
        targetSpeeds = calculateSpeeds();
        request.withSpeeds(targetSpeeds);
    }

    public void setRobotState(Pose2d pose, ChassisSpeeds speeds) {
        this.currentPose = pose;
        this.currentSpeeds = speeds;
    }

    public ChassisSpeeds calculateSpeeds() {
        if (target == null || currentPose == null || currentSpeeds == null) {
            // Safety
            return new ChassisSpeeds();
        }

        // Calculate difference
        var delta = target.getTranslation().minus(currentPose.getTranslation());
        
        // Magnitude target
        double vMagnitude = MathUtil.clamp(
            translationController.setTarget(0.0)
                .setMeasurement(
                    delta.getNorm(), // We are exactly where we are
                    -Util.chassisSpeedsMagnitude(
                        currentSpeeds)) // How fast we are going
                .getOutput(), 
            -MAX_SPEED, MAX_SPEED);
        
        // The angle we are at relative to the target
        var deltaRotation = delta.getAngle();

        double rotation = MathUtil.clamp(
            thetaController.setTarget(target.getRotation().getRadians()) // Theta target
                .setMeasurement(
                    currentPose.getRotation().getRadians(), 
                    currentSpeeds.omegaRadiansPerSecond) // We are where we are and we are as fast as how fast we are going
                .getOutput(),
            -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        SmartDashboard.putNumber("Debug/PIDToPose/vx", vMagnitude * deltaRotation.getCos());
        SmartDashboard.putNumber("Debug/PIDToPose/vy", vMagnitude * deltaRotation.getSin());
        SmartDashboard.putNumber("Debug/PIDToPose/vrotation", rotation);

        return new ChassisSpeeds(
            vMagnitude * deltaRotation.getCos(), // convert from polar to rectangular
            vMagnitude * deltaRotation.getSin(),
            rotation);
    }

    @Override
    public boolean isFinished() {
        return translationController.getError() < EPSILON_TRANSLATION
            && MathUtil.isNear(thetaController.getError(), 0, EPSILON_ROTATION); // Within tolerance
    }

    @Override
    public void end(boolean interrupted) {
        // Swaps out the drive request to a default robot oriented request
        drive.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
    }

    public static double estimateTimeToPose(
        Pose2d currentPose,
        ChassisSpeeds currentSpeeds,
        Pose2d targetPose
    ) {
        double translationError = 
            currentPose.getTranslation().getDistance(targetPose.getTranslation());

        double currentV = Util.chassisSpeedsMagnitude(currentSpeeds);

        return Util.trapezoidProfileTimeToTarget(
            translationError, 
            currentV, 
            0, 
            MAX_SPEED, 
            MAX_ACCEL);
    }

    /** Helper for retrieving the target of this PID to Pose command */
    public Pose2d getTarget() {
        return target;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
