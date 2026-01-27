package frc.robot.lib.control;

import static frc.robot.Robot.controller;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.autos.modes.TeleopAutoMode;

public class Controller { //possibly extends [control]
    //private TeleopAutoMode mTeleopAutoMode = null;
    private SwerveDrive m_SwerveDrive = null;

    private final EventLoop m_loop = new EventLoop();

    public Controller(
        //TeleopAutoMode teleopAutoMode,
    ) {
        //mTeleopAutoMode = teleopAutoMode;
        m_SwerveDrive = SwerveDrive.getInstance();
    }

    public void processKeyCommand() {
        /*if (mTeleopAutoMode == null)
            return;*/

        m_loop.poll();

        if (controller.getPOV() == 90) {
            m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0, -0.4, 0));
        } else if (controller.getPOV() == 0) {
            m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0.4, 0, 0));
        } else if (controller.getPOV() == 270) {
            m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    0, 0.4, 0));
        } else if (controller.getPOV() == 180) {
            m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                    -0.4, 0, 0));
        }
    }
}