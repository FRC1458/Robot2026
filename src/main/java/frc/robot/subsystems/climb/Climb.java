package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climb.ClimbConstants.CARRIAGE_WEIGHT;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMB_MOTOR_ID;
import static frc.robot.subsystems.climb.ClimbConstants.END_EFFECTOR_HEIGHT;
import static frc.robot.subsystems.climb.ClimbConstants.EPSILON;
import static frc.robot.subsystems.climb.ClimbConstants.GEAR_RATIO;
import static frc.robot.subsystems.climb.ClimbConstants.SPROCKET_RADIUS;
import static frc.robot.subsystems.climb.ClimbConstants.getConfig;
import static frc.robot.subsystems.climb.ClimbConstants.metersToRotations;
import static frc.robot.subsystems.climb.ClimbConstants.rotationsToMeters;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.climb.ClimbConstants.Setpoint;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class Climb extends SubsystemBase {
	private static Climb climbInstance;
	public static Climb getInstance() {
		if (climbInstance == null) {
			climbInstance = new Climb();
		}
		return climbInstance;
	}

	// Simulation loop period. Matches the default command-scheduler period,
	// so the sim advances at the same rate periodic() is actually called.
	private static final double SIM_DT_SECONDS = 0.02;

	private final TalonFX climbMotor;

	private double lastReadHeight;
	private double lastReadSpeed;

	private double targetHeight = END_EFFECTOR_HEIGHT;
	private ControlRequest request = new NeutralOut();

	private final ElevatorSim sim = new ElevatorSim(
		DCMotor.getKrakenX60(1),
		GEAR_RATIO,
		CARRIAGE_WEIGHT,
		SPROCKET_RADIUS,
		Setpoint.BASE.height,
		Setpoint.UP.height,
		true,
		Setpoint.BASE.height,
		0.0, 0.0
	);

	private Climb() {
		super();
		climbMotor = new TalonFX(CLIMB_MOTOR_ID);
		climbMotor.getConfigurator().apply(getConfig());
		climbMotor.setNeutralMode(NeutralModeValue.Brake);
		TelemetryManager.getInstance().addSendable(this);
		setDefaultCommand(stop());
	}

	@Override
	public void periodic() {
		// Read the height from the motor encoder
		lastReadHeight = rotationsToMeters(
			climbMotor.getPosition().getValueAsDouble());
		lastReadSpeed = rotationsToMeters(
			climbMotor.getVelocity().getValueAsDouble());
		// updates the motor
		climbMotor.setControl(request);

		if (RobotBase.isSimulation()) {
			updateSimulation();
		}
	}

	/** Steps the elevator physics sim and feeds the result back into the TalonFX's sim state. */
	private void updateSimulation() {
		TalonFXSimState motorSim = climbMotor.getSimState();
		motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		sim.setInputVoltage(motorSim.getMotorVoltage());
		sim.update(SIM_DT_SECONDS);

		// metersToRotations()/rotationsToMeters() are calibrated straight off the
		// raw rotor encoder (METERS_PER_ROTATION was measured against it, with no
		// SensorToMechanismRatio configured), so they already bake in GEAR_RATIO.
		// Do NOT multiply by GEAR_RATIO again here - ElevatorSim gets the real
		// gearing separately, above, for its own torque/dynamics calculation.
		motorSim.setRawRotorPosition(metersToRotations(sim.getPositionMeters()));
		motorSim.setRotorVelocity(metersToRotations(sim.getVelocityMetersPerSecond()));
	}

	/** Swaps the control request */
	private void setRequest(ControlRequest request) {
		this.request = request;
	}

	public Command moveToScoringHeight(Setpoint height) {
		return moveToTarget(height.height).withName(height.name() + ": Move To Height");
	}

	/** Attempts to move the end effector to a height, in meters */
	private Command moveToTarget(double targetHeight) {
		return runOnce(
			() -> {
				this.targetHeight = targetHeight;
				setRequest(new MotionMagicVoltage(
					metersToRotations(
						targetHeight)));
			}
		).andThen(
			Commands.waitUntil(() -> isNearTarget())
		).withName(String.format("%.2f: Moving", targetHeight));
	}

	/** Stops the elevator */
	public Command stop() {
		return runOnce(
			() -> setRequest(
				new PositionVoltage(
					metersToRotations(targetHeight))))
			.withName("Stopped");
	}

	/**
	 * Full climb sequence: drives the end effector up to the UP setpoint to hook
	 * onto the bar, waits until it gets there, then drives back down to BASE,
	 * which pulls the robot up off the ground. Runs as a single Command so the
	 * scheduler never inserts the default (hold-position) command between the
	 * two stages.
	 */
	public Command climb() {
		return moveToScoringHeight(Setpoint.UP)
			.andThen(moveToScoringHeight(Setpoint.BASE))
			.withName("Climb: Hang");
	}

	/**
	 * @deprecated use {@link #climb()} on the instance instead. Kept only so
	 * existing bindings (e.g. in RobotContainer) that call
	 * {@code Climb.HangCommand()} keep compiling.
	 */
	@Deprecated
	public static Command HangCommand() {
		return getInstance().climb();
	}

	/** Sysid commands
	 * @param dynamic If true, then runs dynamic test. If false, quasistatic
	 */	
	public Command sysId(boolean dynamic, SysIdRoutine.Direction direction) {
		return defer(() -> {
			VoltageOut request = new VoltageOut(0);
			// VoltageOut.withOutput(...) returns a NEW VoltageOut rather than
			// mutating in place, so we track the latest applied voltage
			// ourselves for logging and actually push it to the motor here -
			// the original code discarded the result of withOutput() and
			// never called setControl(), so sysId never drove the mechanism.
			double[] appliedVolts = { 0.0 };
			SysIdRoutine sysIdRoutine = new SysIdRoutine(
				new SysIdRoutine.Config(
					null, // Default ramp rate (1 V)
					null, // Default step voltage (7 V)
					null // Use default timeout (10 s)
				),
				new SysIdRoutine.Mechanism(
					output -> {
						appliedVolts[0] = output.in(Volts);
						climbMotor.setControl(request.withOutput(appliedVolts[0]));
					},
                	log -> {
						log.motor("climbMotor")
							.voltage(Volts.of(appliedVolts[0]))
							.linearPosition(
								Meters.of(
									rotationsToMeters(
										climbMotor.getPosition().getValueAsDouble())))
							.linearVelocity(
								MetersPerSecond.of(
									rotationsToMeters(climbMotor.getVelocity().getValueAsDouble())));
					},
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
	public boolean isNearTarget() {
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
		builder.addDoubleProperty(
			"Speed",
			() -> lastReadSpeed,
			null);
		builder.addDoubleProperty(
			"Target Height",
			() -> targetHeight,
			null);
		TelemetryManager.makeSendableTalonFX("ClimbMotor", climbMotor, builder);
	}
}