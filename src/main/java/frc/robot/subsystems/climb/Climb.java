package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

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
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.TelemetryManager;

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

	private double targetHeight = 0;
	private ControlRequest request = new NeutralOut();

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
	}

	/** Swaps the control request */
	private void setRequest(ControlRequest request) {
		this.request = request;
	}

	public Command moveToScoringHeight(Heights height) {
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
		).withName(String.format("%.2f: Unknown, Moving", targetHeight));
	}

	/** Stops the elevator */
	public Command stop() {
		return runOnce(
			() -> setRequest(
				new PositionVoltage(
					metersToRotations(targetHeight))))
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