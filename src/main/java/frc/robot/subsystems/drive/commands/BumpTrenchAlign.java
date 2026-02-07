package frc.robot.subsystems.drive.commands;
import frc.robot.subsystems.drive.Drive;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class BumpTrenchAlign extends Command {
    public boolean activate = false;

    @Override
    public void execute() {
        /* constantly check if controller is activated to align */
        if (activate) {
            /* getting the robot orientation angle */
            var rotation = Drive.getInstance().getPose().getRotation();      
            var actualRotation = getClosestAngle(rotation);
                
        }
    }

    /** Fidning optimal orientation rotation */
    public static Rotation2d getClosestAngle(Rotation2d rotation) {
        Rotation2d angle = Rotation2d.fromRadians((rotation.getRadians() / Constants.TAU * 8) * Constants.TAU / 8);
        return angle;
    }

    /** snapping to closestAngle */
    public void Snap(double targetRadians) {
        /* swerve that gets the robot to turn to angle */
		Drive.getInstance().setSwerveRequest(new SwerveRequest.FieldCentric().withRotationRate(targetRadians));

    }
}
public Command turnToAngle(double targetRadians) {
    return run(() -> {
        double current = getHeadingRadians();

        double omega = thetaPID.calculate(current, targetRadians);

        setSwerveRequest(
            new SwerveRequest.FieldCentric()
                .withRotationalRate(omega)
        );
    }).until(thetaPID::atSetpoint);
}