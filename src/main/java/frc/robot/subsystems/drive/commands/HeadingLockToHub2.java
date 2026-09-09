package frc.robot.subsystems.drive.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.control.ControlConstants.PIDVConstants;
import frc.robot.lib.control.ControlConstants.ProfiledPIDVConstants;
import frc.robot.lib.control.ProfiledPIDVController;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.FieldPoses;

public class HeadingLockToHub2 extends Command {
    private final Drive drive;

    private final SwerveRequest.FieldCentric request;
    private final ProfiledPIDVController thetaController;

    private final Translation2d pose;

    private final Debouncer finishDebouncer;

    public HeadingLockToHub2() {
        this(
            Constants.FieldConstants.allianceCorrected(
                FieldPoses.HUB.pose3d.getTranslation())
                .toTranslation2d());
    }

	/** 
	 * Locks the robot onto a pose. 
	 * Utilizes feedforwards derived from the current chassis speeds
	 */
    public HeadingLockToHub2(Translation2d pose) {
        this(Drive.getInstance(), pose);
    }
    
	/** 
	 * Locks the robot onto a pose. 
	 * Utilizes feedforwards derived from the current chassis speeds
	 */
	public HeadingLockToHub2(Drive drive, Translation2d pose) {
        this.pose = pose;
        this.drive = drive;
        request = 
            new SwerveRequest.FieldCentric();
        
        thetaController = 
            new ProfiledPIDVController(
                new ProfiledPIDVConstants(
                    new PIDVConstants(10.0, 0.0, 1), 
                    new TrapezoidProfile.Constraints(Math.PI * 16, Math.PI * 5))
            );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        finishDebouncer = new Debouncer(0.040, DebounceType.kRising);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        request.withVelocityX(0).withVelocityY(0)
            .withRotationalRate(0);
        drive.setSwerveRequest(request);
        
        thetaController.setInitialSetpoint(
            drive.getPose().getRotation().getRadians(), 
            drive.getState().Speeds.omegaRadiansPerSecond);
    }
    
    @Override
    public void execute() {
        double xDesiredRaw = -Robot.controller.getLeftY();
        double yDesiredRaw = -Robot.controller.getLeftX();

        double[] xy = Util.applyRadialDeadband(xDesiredRaw, yDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);
        double xFancy = xy[0];
        double yFancy = xy[1];

        var state = drive.getState();
        var delta = pose.minus(drive.getPose().getTranslation());
        var targetDirection = delta.getAngle();
        
        var normSq = delta.getNorm() * delta.getNorm();
        var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, drive.getPose().getRotation());
        var rotationalRate = normSq > 1e-4 ? 
            (-delta.getX() * fieldSpeeds.vyMetersPerSecond
            + delta.getY() * fieldSpeeds.vxMetersPerSecond)
            / (normSq) : 0.0;
        
        var rotation = thetaController
            .setTarget(targetDirection.getRadians(), rotationalRate)
            .setMeasurement(state.Pose.getRotation().getRadians(), state.Speeds.omegaRadiansPerSecond)
            .getOutput();

        SmartDashboard.putNumber("error tracking", 
            MathUtil.inputModulus(state.Pose.getRotation().minus(targetDirection).getDegrees(), -180, 180
        ));

        request
            // .withHeadingPID(p.get(), i.get(), d.get())
            .withVelocityX(xFancy * MAX_SPEED)
            .withVelocityY(yFancy * MAX_SPEED)
            .withRotationalRate(rotation);
    }
	
    @Override
    public boolean isFinished() {
        return finishDebouncer.calculate(
            MathUtil.isNear(thetaController.getError(), 0, EPSILON_ROTATION)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.setSwerveRequest(new SwerveRequest.ApplyRobotSpeeds());
        }
    }
}
