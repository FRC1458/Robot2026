package frc.robot;

import frc.robot.auto.AutoSelector;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.VisionDeviceManager;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@SuppressWarnings("unused")
public class Robot extends TimedRobot {
	private static final CommandScheduler commandScheduler = CommandScheduler.getInstance();
	private AutoSelector autoChooser;
	private Command autoCommand;

	public static final CommandXboxController controller =
		new CommandXboxController(Controllers.DRIVER_CONTROLLER_PORT);
		
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		if (Robot.isReal()) {
			VisionDeviceManager.getInstance();
		}
		Drive.getInstance();

		Intake.getInstance();

		TelemetryManager.getInstance();
		FollowPathCommand.warmupCommand().schedule();
		autoChooser = new AutoSelector();

		//robot data loggers 
		boolean usbPresent = new java.io.File("/u").exists();
		if (usbPresent) {
		  DataLogManager.start("/u/logs");  // USB stick
		  System.out.println("Log/USB mounts OK");
		} else {
		  DataLogManager.start();           // falls back to /home/lvuser/logs
		  System.out.println("Log/USB mounts NOT OK");
		}
		DriverStation.startDataLog(DataLogManager.getLog());
	}


	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		commandScheduler.run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
	}

	/** This function is called periodically during disabled. */
	@Override
	public void disabledPeriodic() {

	}

	/** This autonomous runs the autonomous command selected. */
	@Override
	public void autonomousInit() {
		autoCommand = autoChooser.getAuto();
		if (autoCommand != null) {
			autoCommand.schedule();
		} else {
			DriverStation.reportWarning("Tried to schedule a null auto", false);
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

	/** This function is called when autonomous mode ends. */
	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running. 
		if (autoCommand != null) {
			autoCommand.cancel();
		}
		Drive.getInstance().setDefaultCommand(Drive.getInstance().teleopCommand());
		
		ControlsMapping.mapTeleopCommand();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();

		//map test commands
		ControlsMapping.mapSysId();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}
