package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

import static frc.robot.subsystems.drive.DriveConstants.MAX_ROTATION_SPEED;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.therekrab.autopilot.APTarget;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.field.FieldLayout;
import frc.robot.lib.trajectory.LocalADStarWrapper;
import frc.robot.lib.util.TunableNumber;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.ctre.CtreDriveConstants;
import frc.robot.subsystems.drive.commands.AutopilotCommand;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.commands.TrajectoryCommand;
import frc.robot.subsystems.drive.ctre.CtreDrive;
import frc.robot.subsystems.drive.ctre.CtreDriveTelemetry;
import frc.robot.subsystems.vision.VisionConstants;
public class ColorFilter {
    public ColorFilter() {

        // I really hope this works (it won't)
        //needs to get result of the density pipeline

        // var Lresult = VisionDeviceManager.getInstance().getFrontLVision().getResult();
        // var Rresult = VisionDeviceManager.getInstance().getFrontRVision().getResult();
        
    }

    //by some magic we obtain a point on the camera.
    double xcoord = 0.0; //from [-1.0 to 1.0]
    double ycoord = 0.0; //from [-1.0 to 1.0]

    //we want to discord targets with a ycoord that would result in it
    //going to the other team's side
    double ylimit;
    double desiredrotation = xcoord * VisionConstants.FOVHOR;
    double distancefromtarget = 0.1; // should change with camera idk
    Rotation2d m_angle = Drive.getInstance().getPose().getRotation();
    Rotation2d desiredangle = Rotation2d.fromDegrees(desiredrotation).plus(m_angle);
    double desiredx = Drive.getInstance().getPose().getX() + distancefromtarget * Math.sin((Rotation2d.fromDegrees(desiredrotation).plus(m_angle)).getRadians());
    double desiredy = Drive.getInstance().getPose().getY() + distancefromtarget * Math.sin((Rotation2d.fromDegrees(desiredrotation).plus(m_angle)).getRadians());
    public Command goToDesired() {
        //sequentialcommandgroup?  
        return runOnce(() -> {
            
            Drive.setSwerveRequest(turnRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(DriveConstants.MAX_ROTATION_SPEED));
        }
        ).withTimeout(desiredrotation * (Math.PI / 180) / DriveConstants.MAX_ROTATION_SPEED);
    }
}
