package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Controllers;
import frc.robot.auto.AutoSelector;
import frc.robot.lib.util.MovingAverageDouble;

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
    private final SendableChooser<String> mapChooser = new SendableChooser<>();

	public static final CommandXboxController controller =
		new CommandXboxController(Controllers.DRIVER_CONTROLLER_PORT);

	public double lastTime = -1.0;
	public MovingAverageDouble fpsTracker = new MovingAverageDouble(60);
		
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		
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
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
	}

	/** This function is called periodically during disabled. */
	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void disabledExit() {}

	/** This autonomous runs the autonomous command selected. */
	@Override
	public void autonomousInit() {
		autoCommand = autoChooser.getAuto();
		if (autoCommand != null) {
			commandScheduler.schedule(autoCommand);
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
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}


	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
		
	}
}
