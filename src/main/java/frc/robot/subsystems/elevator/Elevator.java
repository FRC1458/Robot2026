package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.coralshooter.CoralShooter;
import frc.robot.subsystems.led.Led;

public class Elevator extends SubsystemBase {
	private static Elevator elevatorInstance;
	public static Elevator getInstance() {
		if (elevatorInstance == null) {
			elevatorInstance = new Elevator();
		}
		return elevatorInstance;
	}

	private final TalonFX leftMotor;
	private final TalonFX rightMotor;

	private double lastReadHeight;
	private ControlRequest request = new NeutralOut();

	private ElevatorSim elevatorSim;
	private Mechanism2d mechanism;
	private MechanismLigament2d ligament;

	private Elevator() {
		super();
		leftMotor = new TalonFX(Motors.LEFT.id);
		rightMotor = new TalonFX(Motors.RIGHT.id);
		leftMotor.getConfigurator().apply(getConfig());
		rightMotor.getConfigurator().apply(getConfig());
		leftMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setControl(
			new Follower(leftMotor.getDeviceID(), true));
		if (Robot.isSimulation()) {
			elevatorSim = new ElevatorSim(
				DCMotor.getKrakenX60(2),
				GEAR_RATIO,
				CARRIAGE_WEIGHT,
				SPROCKET_RADIUS,
				0.0,
				2.0,
				true,
				0.01, 
				0.0001, 0.0
			);
		}
			
		mechanism = new Mechanism2d(1, 3);
		MechanismRoot2d root = mechanism.getRoot("Elevator", 0.5, 0);
		ligament = new MechanismLigament2d("Elevator", 0, 90);
		root.append(ligament);
		SmartDashboard.putData("Mechanisms/Elevator", mechanism);
		TelemetryManager.getInstance().addSendable(this);
		setDefaultCommand(stop());
	}

	@Override
	public void periodic() {
		// Read the height from the motor encoder
		lastReadHeight = rotationsToMeters(
			leftMotor.getPosition().getValueAsDouble()) 
			+ END_EFFECTOR_HEIGHT;
		// updates the motor
		leftMotor.setControl(request);
		// Only if not in simulation: sets the ligament manually
		if (Robot.isReal()) {
			ligament.setLength(lastReadHeight - END_EFFECTOR_HEIGHT);
		}
	}

	@Override
	public void simulationPeriodic() {
		// sets the voltages
		TalonFXSimState simState = leftMotor.getSimState();
		simState.setSupplyVoltage(RobotController.getBatteryVoltage());

		// updates the elevator sim
		double motorVoltage = simState.getMotorVoltageMeasure().in(Units.Volts);
		elevatorSim.setInput(motorVoltage);
		elevatorSim.update(Constants.DT);

		double motorRot = metersToRotations(elevatorSim.getPositionMeters());
		double motorVel = metersToRotations(elevatorSim.getVelocityMetersPerSecond());

		// Feed correct motor values into CTRE sim
		simState.setRawRotorPosition(motorRot);
		simState.setRotorVelocity(motorVel);

		// Updates the battery
		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

		// updates the widget
		ligament.setLength(elevatorSim.getPositionMeters());
	}

	/** Swaps the control request */
	private void setRequest(ControlRequest request) {
		this.request = request;
	}

	/** Moves to a known scoring height */
	public Command moveToScoringHeight(Heights height) {
		return defer(() -> {
			if (!CoralShooter.getInstance().isCoralObstructingElevator()) {
				return moveToScoringHeightUnsafe(height);
			} else {
				DriverStation.reportWarning(
					"Tried to move elevator while a coral is obstructing",
					false);
				return Commands.parallel(
					stop(), 
					Led.getInstance().blinkCommand(Color.kRed, Color.kBlack, 0.12, 0.5));
			}
		}).withName(height.name() + ": Safe, Moving");
	}

	/** 
	 * Moves to a known scoring height 
	 * <p> is not safe </p>
	*/
	private Command moveToScoringHeightUnsafe(Heights height) {
		return moveToTarget(height.height).withName(height.name() + ": Unsafe, Moving");
	}

	/** Attempts to move the end effector to a height, in meters */
	private Command moveToTarget(double targetHeight) {
		return runOnce(() -> setRequest(
			new MotionMagicVoltage(
				metersToRotations(
					targetHeight - END_EFFECTOR_HEIGHT))))
			.andThen(
				Commands.waitUntil(() -> isNearTarget(targetHeight)))
			.withName(String.format("%.2f: Unknown, Moving", targetHeight));
	}

	/** Stops the elevator */
	public Command stop() {
		return runOnce(
			() -> setRequest(
				new PositionVoltage(leftMotor.getPosition().getValue())))
			.withName("Stopped");
	}

	/** Sysid commands
	 * @param dynamic If true, then runs dynamic test. If false, quasistatic
	 */
	public Command sysId(boolean dynamic, SysIdRoutine.Direction direction) {
		return defer(() -> {
			VoltageOut request = new VoltageOut(0);
			SysIdRoutine sysIdRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
					null, // Default ramp rate (1 V)
					null, // Default step voltage (7 V)
					null, // Use default timeout (10 s)
					// Log state with SignalLogger class
					state -> SignalLogger.writeString("SysIdElevator_State", state.toString())
				),
				new SysIdRoutine.Mechanism(
					output -> request.withOutput(output),
					null,
					this
				)
			);
			if (dynamic) {
				return sysIdRoutine.dynamic(direction);
			} else {
				return sysIdRoutine.quasistatic(direction);
			}
		});
	}

	/** Checks if the end effector is within 1 cm of the target */
	public boolean isNearTarget(double targetHeight) {
		return MathUtil.isNear(
			lastReadHeight,
			targetHeight,
			EPSILON);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty(
			"Height",
			() -> lastReadHeight,
			null);
		TelemetryManager.makeSendableTalonFX("Left", leftMotor, builder);
		TelemetryManager.makeSendableTalonFX("Right", rightMotor, builder);
	}
}