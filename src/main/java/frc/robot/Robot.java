package frc.robot;

import frc.robot.auto.AutoSelector;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.VisionDeviceManager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */

 

public class Robot extends LoggedRobot {
	private static final CommandScheduler commandScheduler = CommandScheduler.getInstance();
	public AutoSelector autoChooser;
	private Command autoCommand;

	public static final CommandXboxController controller =
		new CommandXboxController(Controllers.DRIVER_CONTROLLER_PORT);
		
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		if (Robot.isSimulation()) {
			// TODO: remove this
			
		}
		
		// RobotState.reset(Timer.getFPGATimestamp(), new Pose2d());
		// RobotState.resetKalman();

		Drive.getInstance();
		if (Robot.isReal()) {
			VisionDeviceManager.getInstance();
		}
		TelemetryManager.getInstance();

		FollowPathCommand.warmupCommand().schedule();;

		autoChooser = new AutoSelector();

		Logger.recordMetadata("Newlog", "CurrentLog"); // Set a metadata value

		if (isReal()) {
			Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    		Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
		} else {
    		setUseTiming(false); // Run as fast as possible
    		String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    		Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    		Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
}

		Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    	// ...

    	Logger.start();



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
		// RobotState.setAlliance(DriverStation.getAlliance());
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
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
    	DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}

	
}
