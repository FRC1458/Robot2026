package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climb.ClimbConstants.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.lib.io.TalonFXIO;
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

	private final TalonFX climbMotor;

	private double lastReadHeight;
	private double lastReadSpeed;

	private double targetHeight = END_EFFECTOR_HEIGHT;
	private ControlRequest request = new NeutralOut();

	private ElevatorSim sim;

	private ClimbIO io;

	private Climb() {
		super();
		climbMotor = new TalonFX(CLIMB_MOTOR_ID);
		climbMotor.getConfigurator().apply(getConfig());
		climbMotor.setNeutralMode(NeutralModeValue.Brake);
		if (Robot.isSimulation()) {
			sim = new ElevatorSim(
				DCMotor.getKrakenX60(1),
				GEAR_RATIO,
				1.13,
				0.05,
				0.0,
				1,
				true,
				0.0,
				0.0, 0.0
			);
		}
		io = new ClimbIO(getName(), climbMotor);
		// TelemetryManager.getInstance().addSendable(this);
	}

	@Override
	public void periodic() {
		// Read the height from the motor encoder
		lastReadHeight = 
			climbMotor.getPosition().getValueAsDouble();
		lastReadSpeed = 
			climbMotor.getVelocity().getValueAsDouble();
		// updates the motor
		climbMotor.setControl(request);

		io.updateInputs(lastReadHeight, lastReadSpeed, getCurrentCommand(), getDefaultCommand());
		io.process();
	}

	@Override
	public void simulationPeriodic() {
		climbMotor.getSimState().setSupplyVoltage(12);
		sim.setInput(climbMotor.getSimState().getMotorVoltage());
		sim.update(0.020);
		climbMotor.getSimState().setRawRotorPosition(sim.getPositionMeters() * CONVERSION_FACTOR * 1 / Constants.TAU);
		climbMotor.getSimState().setRotorVelocity(
			sim.getVelocityMetersPerSecond() * CONVERSION_FACTOR * 1 / Constants.TAU);
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
						targetHeight));
			}
		).andThen(
			Commands.waitUntil(() -> isNearTarget())
		).withName(String.format("%.2f: Unknown, Moving", targetHeight));
	}

	/** Stops the elevator */
	public Command stop() {
		return runOnce(
			() -> setRequest(
				new NeutralOut()))
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
					null // Use default timeout (10 s)
				),
				new SysIdRoutine.Mechanism(
					output -> request.withOutput(output),					
                	log -> {
						log.motor("climbMotor")
							.voltage(Volts.of(request.Output))
							.linearPosition(
								Meters.of(
									climbMotor.getPosition().getValueAsDouble()))
							.linearVelocity(
								MetersPerSecond.of(
									climbMotor.getVelocity().getValueAsDouble()));
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

	// command

	public Command hangCommand() {
		return moveToScoringHeight(Setpoint.UP)
			.andThen(Climb.getInstance().moveToScoringHeight(Setpoint.BASE));
	}
}