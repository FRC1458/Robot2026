package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.LEFT_OFFSET;
import static frc.robot.subsystems.shooter.ShooterConstants.OFFSET;
import static frc.robot.subsystems.shooter.ShooterConstants.RIGHT_OFFSET;
import static frc.robot.subsystems.shooter.ShooterConstants.rotation;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Controllers;
import frc.robot.auto.AutoSelector;
import frc.robot.lib.field.Sotm;
import frc.robot.lib.subsystem.LoggedSubsystem;
import frc.robot.lib.util.FuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.LeftIndexer;
import frc.robot.subsystems.indexer.RightIndexer;
import frc.robot.subsystems.indexer.Roller;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.VisionDeviceManager;

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
	public final Flywheel shooterBL;
	public final VisionDeviceManager visionDeviceManager;
	public final Hood hood;

	private final Notifier allianceThread;

	public Visualizer visualizer;
	public FuelSim fuelSim;
	private int hopper = 0;
	private Timer lastShotLeft = new Timer();
	private Timer lastShotRight = new Timer();

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		robotInstance = this;
		autoChooser = new AutoSelector();
		DogLog.setOptions(
				new DogLogOptions().withCaptureDs(true).withLogExtras(true).withNtTunables(true));
		allianceThread =
				new Notifier(
						() -> {
							while (true) {
								if (DriverStation.getAlliance().isPresent()) {
									if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
										Constants.isBlue = false;
										Constants.FieldConstants.hubLocation =
												Constants.FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
									} else {
										Constants.isBlue = true;
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
						});
		allianceThread.startSingle(0);

		drive = new Drive();
		intakePivot = new IntakePivot();
		intakeRoller = new IntakeRoller();
		intake = new Intake(intakePivot, intakeRoller);
		leftIndexer = new LeftIndexer();
		rightIndexer = new RightIndexer();
		roller = new Roller();
		indexer = new Indexer(leftIndexer, rightIndexer, roller);
		shooterBL = new Flywheel();
		hood = new Hood();
		shooter = new Shooter(shooterBL, hood);
		visionDeviceManager = new VisionDeviceManager(drive);

		ControlsMapping.bind();

		if (Robot.isSimulation()) {
			fuelSim = new FuelSim();
			fuelSim.spawnStartingFuel();
			fuelSim.registerIntake(
					Inches.of(-24.248542),
					Inches.of(-16.625000),
					Inches.of(-12.687500),
					Inches.of(10.250000),
					() ->
							intakePivot.getIo().getPosition().lt(Degrees.of(5))
									&& intakeRoller.getIo().getVelocity().gt(RotationsPerSecond.of(10))
									&& hopper < 560,
					() -> hopper++);
			fuelSim.registerRobot(
					Inches.of(33), Inches.of(33), Inches.of(10.36), drive::getPose, drive::getFieldSpeeds);
			fuelSim.start();
			fuelSim.setLoggingFrequency(50);
			fuelSim.setSubticks(5);
		}

		visualizer = new Visualizer(() -> new Pose3d(drive.getPose()), drive::getFieldSpeeds);
		sotm = drive.sotm;
	}

	Sotm sotm; 

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
		// System.out.println(speed);

		Translation2d solution = sotm.solve(drive.getPose().getTranslation(), Constants.FieldConstants.hubLocation, new Translation2d(drive.getFieldSpeeds().vxMetersPerSecond, drive.getFieldSpeeds().vyMetersPerSecond));
		
		Translation2d delta = solution.minus(drive.getPose().getTranslation());

		AngularVelocity shooterVelocity = RotationsPerSecond.of(ShooterConstants.VEL_MAP.get(delta.getNorm()));

		Distance wheelRadius = Inches.of(4).div(2.0).plus(Inches.of(1.5).div(2.0)).div(2.0);
		LinearVelocity speed = MetersPerSecond.of(shooterVelocity.in(RadiansPerSecond) * (wheelRadius).in(Meters)).times(0.95);

		Angle angle = Degrees.of(ShooterConstants.ANGLE_MAP.get(delta.getNorm()));

		visualizer.updateFuel(speed, angle, delta.getAngle());
		DogLog.log("Virtual Target", new Pose3d(new Pose2d(solution, Rotation2d.kZero)));
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
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();
	}

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
	public void simulationInit() {
		lastShotLeft.start();
		lastShotRight.start();
	}

	@Override
	public void simulationPeriodic() {
		DogLog.log("Hopper", hopper);
		fuelSim.updateSim();

		double randomness = 0.0 / 180.0 * Math.PI;
		var random1 =
				new Rotation3d(
						randomness * 2 * (0.5 - Math.random()),
						randomness * 2 * (0.5 - Math.random()),
						randomness * 2 * (0.5 - Math.random()));


		if (rightIndexer.getIo().getVelocity().gt(RotationsPerSecond.of(10))) {
			if (hopper > 0) {
				int shots = (int) Math.round(0.02 * Math.random() * 2 * 20);
				for (int i = 0; i < shots; i++) {
					var random2 = new Translation3d(0, 2 * (0.5 - Math.random()) * Inches.of(18-5.91).in(Meters), 0);
					double shooterVelocity = shooter.getVelocity().in(RadiansPerSecond);

					double wheelRadius = (Inches.of(4).in(Meters) / 2.0 + Inches.of(1.5).in(Meters) / 2.0) / 2.0;
					double speed = shooterVelocity * wheelRadius * 0.99;

					Translation3d shotPosition =
							new Pose3d(drive.getPose())
									.plus(new Transform3d(OFFSET, Rotation3d.kZero))
									.plus(new Transform3d(random2, Rotation3d.kZero))
									.getTranslation();

					fuelSim.spawnFuel(
							shotPosition,
							new Translation3d(
									speed, 
									new Rotation3d(Radians.of(0), Degrees.of(-90).minus(shooter.getAngle()), Radians.of(0))
										.plus(new Rotation3d(drive.getPose().getRotation())).plus(random1)).plus(
											new Translation3d(drive.getFieldSpeeds().vxMetersPerSecond, drive.getFieldSpeeds().vyMetersPerSecond, 0)));

					lastShotRight.reset();

					hopper--;
				}
			}
		}

		DogLog.log("Blue Score", FuelSim.Hub.BLUE_HUB.getScore());
		DogLog.log("Red Score", FuelSim.Hub.RED_HUB.getScore());
	}
}
