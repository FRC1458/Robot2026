package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Controllers;
import frc.robot.auto.AutoSelector;
import frc.robot.lib.subsystem.LoggedSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.LeftIndexer;
import frc.robot.subsystems.indexer.RightIndexer;
import frc.robot.subsystems.indexer.Roller;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterBL;
import frc.robot.subsystems.shooter.ShooterBR;
import frc.robot.subsystems.shooter.ShooterTL;
import frc.robot.subsystems.shooter.ShooterTR;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@SuppressWarnings("unused")
public class Robot extends TimedRobot {
	public static Robot robotInstance;

	public static Robot getInstance() {
		return robotInstance;
	}

	private AutoSelector autoChooser;
	private Command autoCommand;
	private final SendableChooser<String> mapChooser = new SendableChooser<>();

	public static final CommandXboxController controller =
			new CommandXboxController(Controllers.DRIVER_CONTROLLER_PORT);

	public final Drive drive;
	public final Intake intake;
	public final IntakePivot intakePivot;
	public final IntakeRoller intakeRoller;
	public final Indexer indexer;
	public final LeftIndexer leftIndexer;
	public final RightIndexer rightIndexer;
	public final Roller roller;
	public final Shooter shooter;
	public final ShooterBL shooterBL;
	public final ShooterBR shooterBR;
	public final ShooterTL shooterTL;
	public final ShooterTR shooterTR;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		robotInstance = this;
		autoChooser = new AutoSelector();
		DogLog.setOptions(
				new DogLogOptions().withCaptureDs(true).withLogExtras(true).withNtTunables(true));
		try (Notifier thread =
				new Notifier(
						() -> {
							while (true) {
								if (DriverStation.getAlliance().isPresent()) {
									if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
										Constants.FieldConstants.hubLocation =
												Constants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
									} else {
										Constants.FieldConstants.hubLocation =
												Constants.FieldConstants.Hub.topCenterPoint.toTranslation2d();
									}
								}

								try {
									Thread.sleep(100);
								} catch (InterruptedException e) {
									e.printStackTrace();
								}
							}
						})) {
			thread.startSingle(0);
		}

		drive = new Drive();
		intakePivot = new IntakePivot();
		intakeRoller = new IntakeRoller();
		intake = new Intake(intakePivot, intakeRoller);
		leftIndexer = new LeftIndexer();
		rightIndexer = new RightIndexer();
		roller = new Roller();
		indexer = new Indexer(leftIndexer, rightIndexer, roller);
		shooterBL = new ShooterBL();
		shooterBR = new ShooterBR();
		shooterTL = new ShooterTL();
		shooterTR = new ShooterTR();
		shooter = new Shooter(shooterBL, shooterBR, shooterTL, shooterTR);

		ControlsMapping.bind();
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
		LoggedSubsystem.logAll();
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {}

	/** This function is called periodically during disabled. */
	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	/** This autonomous runs the autonomous command selected. */
	@Override
	public void autonomousInit() {
		autoCommand = autoChooser.getAuto();
		if (autoCommand != null) {
			CommandScheduler.getInstance().schedule(autoCommand);
		} else {
			DriverStation.reportWarning("Tried to schedule a null auto", false);
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	/** This function is called when autonomous mode ends. */
	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}
}
