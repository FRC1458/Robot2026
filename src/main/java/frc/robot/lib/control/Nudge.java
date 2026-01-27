package frc.robot.lib.control;
/*
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List; */

import static frc.robot.Robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants; //62 - 67
//import frc.robot.FieldLayout;
import frc.robot.RobotState; //97
//import frc.robot.autos.actions.*;
import frc.robot.autos.modes.TeleopAutoMode; //27, 43, 47, 57-59
import frc.robot.lib.util.Util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Controller { //possibly extends [control]
    private XboxController mXboxController1 = null; // private final instead, port num
    private TeleopAutoMode mTeleopAutoMode = null;
    private Joystick m_JoyStick = null;
    private SwerveDrive m_SwerveDrive = null;

    private final EventLoop m_loop = new EventLoop();
    
    private boolean isFieldRelative = true;

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    public Controller(
        XboxController xboxController1, 
        TeleopAutoMode teleopAutoMode,
        Joystick joystick
    ) {
        controller = xboxController1;
        mTeleopAutoMode = teleopAutoMode;
        m_JoyStick = joystick;
        m_SwerveDrive = SwerveDrive.getInstance();

        controller.start(m_loop).ifHigh(
            () -> isFieldRelative = !isFieldRelative
        );
    }

    public void processKeyCommand() {
        if (mTeleopAutoMode == null)
            return;

        m_loop.poll();

        double translationVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(translationAxis), Constants.stickDeadband)
                * Constants.SwerveConstants.maxSpeed;
        double strafeVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(strafeAxis), Constants.stickDeadband)
                * Constants.SwerveConstants.maxSpeed;
        double rotationVal = -MathUtil.applyDeadband(m_JoyStick.getRawAxis(rotationAxis), Constants.stickDeadband)
                * Constants.Swerve.maxAngularVelocity;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            translationVal = -translationVal;
            strafeVal = -strafeVal;
        }

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
        } else {
            if (isFieldRelative) {
                m_SwerveDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationVal, strafeVal, rotationVal,
                        Util.robotToFieldRelative(m_SwerveDrive.getHeading(), DriverStation.getAlliance().get() == Alliance.Red)));
            } else {
                m_SwerveDrive.feedTeleopSetpoint(new ChassisSpeeds(
                        translationVal, strafeVal, rotationVal));
            }
        }

        Twist2d velocity = RobotState.getInstance().getMeasuredVelocity();

    }
}