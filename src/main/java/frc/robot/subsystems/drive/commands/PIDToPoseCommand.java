package frc.robot.subsystems.drive.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.util.Util;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * A command that moves the drivetrain to a pose.
 */
public class PIDToPoseCommand extends Command {
    public final Drive drive;

    private final SwerveRequest.ApplyFieldSpeeds request = 
        new SwerveRequest.ApplyFieldSpeeds();

    private final ProfiledPIDVController translationController;
    private final ProfiledPIDVController thetaController;

    private final Pose2d target;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    public PIDToPoseCommand(Pose2d target) {
        this(
            Drive.getInstance(), 
            target,
            DriveConstants.PROFILED_TRANSLATION_CONSTANTS, 
            DriveConstants.ROTATION_CONSTANTS);
    }
    
    public PIDToPoseCommand(Drive drive, Pose2d target, ProfiledPIDFConstants translationConstants, ProfiledPIDFConstants rotationConstants) {
        this.drive = drive;
        this.target = target;
        translationController = new ProfiledPIDVController(translationConstants);
        thetaController = new ProfiledPIDVController(rotationConstants);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
        setName("PID to " + target.toString());
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
        request.withSpeeds(calculateSpeeds());
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
        translationController.setTarget(delta.getNorm());

        translationController.setInput(
            0.0, // We are exactly where we are
            Util.chassisSpeedsMagnitude(
                currentSpeeds)); // How fast we are going
        
        double vMagnitude = translationController.getOutput();
        SmartDashboard.putNumber("Debug/PIDToPoseCommand/vmag", vMagnitude);
        
        // The angle we are at relative to the target
        var deltaRotation = delta.getAngle();

        // Theta target
        thetaController.setTarget(target.getRotation().getRadians());
        // We are where we are and we are as fast as how fast we are going
        thetaController.setInput(
            currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond); 

        double rotation = thetaController.getOutput();

        return new ChassisSpeeds(
            vMagnitude * deltaRotation.getCos(), // convert from polar to rectangular
            vMagnitude * deltaRotation.getSin(),
            rotation);
    }

    @Override
    public boolean isFinished() {
        return translationController.error < DriveConstants.EPSILON_TRANSLATION
            && MathUtil.isNear(thetaController.error, 0, DriveConstants.EPSILON_ROTATION); // Within tolerance
    }

    @Override
    public void end(boolean interrupted) {
        // Swaps out the drive request to a default robot oriented request
        drive.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
    }

    /** Helper for retrieving the target of this PID to Pose command */
    public Pose2d getTarget() {
        return target;
    }
}
