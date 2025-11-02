package frc.robot.subsystems.drive.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDFConstants;
import frc.robot.lib.util.Util;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.subsystems.drive.Drive;

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
            Constants.Auto.PROFILED_TRANSLATION_CONSTANTS, 
            Constants.Auto.ROTATION_CONSTANTS);
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
            return new ChassisSpeeds();
        }

        var delta = target.getTranslation().minus(currentPose.getTranslation());
        
        translationController.setTarget(delta.getNorm());

        translationController.setInput(
            0.0,
            Util.chassisSpeedsMagnitude(
                currentSpeeds));
        
        double vMagnitude = translationController.getOutput();
        SmartDashboard.putNumber("Debug/PIDToPoseCommand/vmag", vMagnitude);
        var deltaRotation = delta.getAngle();

        thetaController.setTarget(target.getRotation().getRadians());
        thetaController.setInput(
            currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);

        double rotation = thetaController.getOutput();

        return new ChassisSpeeds(
            vMagnitude * deltaRotation.getCos(),
            vMagnitude * deltaRotation.getSin(),
            rotation);
    }

    @Override
    public boolean isFinished() {
        return translationController.error < 0.03 
            && MathUtil.isNear(thetaController.error / Math.PI * 180, 0, 3.0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
    }

    public Pose2d getTarget() {
        return target;
    }
}
