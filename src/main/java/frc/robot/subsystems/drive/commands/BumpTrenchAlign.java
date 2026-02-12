package frc.robot.subsystems.drive.commands;
import static frc.robot.subsystems.drive.DriveConstants.*;
import frc.robot.subsystems.drive.Drive;

import frc.robot.Constants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class BumpTrenchAlign extends Command {
    private final SwerveRequest.FieldCentricFacingAngle request = 
    new SwerveRequest.FieldCentricFacingAngle()
    .withHeadingPID(
        ROTATION_CONSTANTS.kP,
        ROTATION_CONSTANTS.kI,
        ROTATION_CONSTANTS.kD);

    private final Drive drive;
    public BumpTrenchAlign(Drive drive) {
        this.drive = drive;
    }

    public BumpTrenchAlign() {
        this(Drive.getInstance());
    }
    
    //just defining orientPitchRadiansInitial so it can be modified in initialize() and used in execute()
    private double orientPitchInitial;

    // boolean to modify isFinished() to affect end()
    private boolean isFinished;
    
    
    @Override
    public void initialize() {
        drive.setSwerveRequest(request);
        var actualRotation = getClosestAngle(drive.getPose().getRotation());
        request.withTargetDirection(actualRotation);
        request.withVelocityX(0);
        
        Rotation3d orientationInitial = drive.getCtreDrive().getRotation3d();
        double orientPitchInitial = orientationInitial.getY();
    }

    @Override
    public void execute() {
        Rotation3d orientation = drive.getCtreDrive().getRotation3d();
        double orientPitch = orientation.getY();

        request.withVelocityX(1.5);
        
        if (orientPitchInitial - 0.1 < orientPitch && orientPitch < orientPitchInitial + 0.1) {
            isFinished = true;
        } else {
            isFinished = false;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    /** ends the align */
    public void end(boolean interrupted) {
        if (!interrupted) {
            drive.addVisionUpdate(
                new Pose2d(
                    // TODO: fill pose2d with current pose, but with x value of over the bump
                ), Timer.getFPGATimestamp(), 
                VecBuilder.fill(0.0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY));
            // Ok so some cursed tech here, but what we can do is that since we know the exact
        }
        drive.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
        // ^ at the end stops the robot
    }

    /** Finding optimal orientation rotation */
    public static Rotation2d getClosestAngle(Rotation2d rotation) {
        Rotation2d angle = Rotation2d.fromRadians(
            (int) (rotation.getRadians() / Constants.TAU * 4.0) 
            * Constants.TAU / 4.0 + Constants.TAU / 8.0);
        return angle;
    }
}