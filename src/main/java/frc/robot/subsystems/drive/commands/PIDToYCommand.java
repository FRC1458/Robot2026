package frc.robot.subsystems.drive.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.control.ControlConstants.*;
import frc.robot.lib.control.*;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.Drive;

/**
 * A command that moves the drivetrain to a pose.
 */
public class PIDToYCommand extends Command {
    public final Drive drive;

    private final SwerveRequest.ApplyFieldSpeeds request = 
        new SwerveRequest.ApplyFieldSpeeds();

    private final ProfiledPIDVController yController;
    private final ProfiledPIDVController thetaController;

    private final double yTarget;
    private final Rotation2d rotTarget;
    private Pose2d currentPose;
    private ChassisSpeeds currentSpeeds;

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    private final Debouncer finishDebouncer;

    public PIDToYCommand(double yTarget, Rotation2d rotTarget) {
        this(
            Drive.getInstance(), 
            yTarget,
            rotTarget,
            PROFILED_TRANSLATION_CONSTANTS, 
            PROFILED_ROTATION_CONSTANTS);
    }
    
    public PIDToYCommand(Drive drive, double yTarget, Rotation2d rotTarget, ProfiledPIDVConstants translationConstants, ProfiledPIDVConstants rotationConstants) {
        this.drive = drive;
        this.yTarget = yTarget;
        this.rotTarget = rotTarget;
        yController = new ProfiledPIDVController(translationConstants);
        thetaController = new ProfiledPIDVController(rotationConstants);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        finishDebouncer = new Debouncer(0.040, DebounceType.kRising);

        addRequirements(drive);
        setName("(" + drive.getPose().getX() + ", " + yTarget + ", " 
            + rotTarget.getDegrees() + " deg)" + " :PID to pose");
    }

    @Override
    public void initialize() {
        var state = drive.getState();
        thetaController.setInitialSetpoint(
            state.Pose.getRotation().getRadians(), 
            state.Speeds.omegaRadiansPerSecond);
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
        if (yTarget == 0 || rotTarget == null || currentPose == null || currentSpeeds == null) {
            // Safety
            return new ChassisSpeeds();
        }

        // Calculate difference
        double deltaY = yTarget - currentPose.getTranslation().getY();
        
        // Magnitude target
        double vMagnitude = -MathUtil.clamp(
            yController.setTarget(0)
                .setMeasurement(
                    deltaY, // We are exactly where we are
                    -Util.chassisSpeedsMagnitude(
                        currentSpeeds)) // How fast we are going
                .getOutput(), 
            -MAX_SPEED, MAX_SPEED);
        
        // The angle we are at relative to the target
        double deltaRotation = rotTarget.getRadians()-currentPose.getRotation().getRadians();

        double rotation = MathUtil.clamp(
            thetaController.setTarget(rotTarget.getRadians()) // Theta target
                .setMeasurement(
                    currentPose.getRotation().getRadians(), 
                    currentSpeeds.omegaRadiansPerSecond) // We are where we are and we are as fast as how fast we are going
                .getOutput(),
            -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        SmartDashboard.putNumber("Debug/PIDToPose/vx", vMagnitude * Math.cos(deltaRotation));
        SmartDashboard.putNumber("Debug/PIDToPose/vy", vMagnitude * Math.sin(deltaRotation));
        SmartDashboard.putNumber("Debug/PIDToPose/vrotation", rotation);

        return new ChassisSpeeds(
            vMagnitude * Math.cos(deltaRotation), // convert from polar to rectangular
            vMagnitude * Math.sin(deltaRotation),
            rotation);
    }

    @Override
    public boolean isFinished() {
        return finishDebouncer.calculate(
            Math.abs(yController.getError()) <= EPSILON_TRANSLATION
                && MathUtil.isNear(thetaController.getError(), 0, EPSILON_ROTATION)); // Within tolerance
    }

    @Override
    public void end(boolean interrupted) {
        // Swaps out the drive request to a default robot oriented request
        drive.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
        System.out.printf(
            "Done with auto-align, error: %.5f m, interrupted: %b\n",
            -yController.getError(), interrupted);
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
    public double getTarget() {
        return yTarget;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }
}
