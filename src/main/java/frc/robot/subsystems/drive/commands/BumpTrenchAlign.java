package frc.robot.subsystems.drive.commands;
import frc.robot.subsystems.drive.Drive;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class BumpTrenchAlign extends Command {
    public boolean activate = false;

    private final PIDController thetaController = new PIDController(5, 0, 0);

    public BumpTrenchAlign() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI); // wraps around
        thetaController.setTolerance(Math.toRadians(2)); // stop when within 2°
    }

    @Override
    public void execute() {
        /* constantly check if controller is activated to align */
        if (activate) {
            /* getting the robot orientation angle */
            var rotation = Drive.getInstance().getPose().getRotation();
            double current = rotation.getRadians();
            double wantedAngle = getClosestAngle(rotation).getRadians();

            double omega = thetaController.calculate(current, wantedAngle);
            Drive.getInstance().setSwerveRequest(new SwerveRequest.FieldCentric().withRotationalRate(omega));
        }
    }

    /** Fidning optimal orientation rotation */
    public static Rotation2d getClosestAngle(Rotation2d rotation) {
        Rotation2d angle = Rotation2d.fromRadians((rotation.getRadians() / Constants.TAU * 8) * Constants.TAU / 8);
        return angle;
    }
    
    @Override
    public boolean isFinished() {
        return thetaController.atSetpoint();
    }
}