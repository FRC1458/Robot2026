package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.TelemetryManager;

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
	private ControlRequest request;

	private DCMotorSim motorSim;
	private ElevatorSim elevatorSim;
	private Mechanism2d mechanism;
	private MechanismLigament2d ligament;

	private Elevator() {
		leftMotor = new TalonFX(ElevatorConstants.Motors.LEFT.id);
		rightMotor = new TalonFX(ElevatorConstants.Motors.RIGHT.id);
		leftMotor.getConfigurator().apply(ElevatorConstants.getConfig());
		rightMotor.getConfigurator().apply(ElevatorConstants.getConfig());
		rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
		leftMotor.setNeutralMode(NeutralModeValue.Brake);

		if (Robot.isSimulation()) {
			motorSim = new DCMotorSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(2), 0.001, ElevatorConstants.GEAR_RATIO),
					DCMotor.getKrakenX60Foc(2));
			elevatorSim = new ElevatorSim(
				LinearSystemId.createElevatorSystem(
					motorSim.getGearbox(),
					6.55,
					ElevatorConstants.SPROCKET_RADIUS,
					ElevatorConstants.GEAR_RATIO
				),
				motorSim.getGearbox(),
				0.0,
				2.0,
				true,
				0.0,
				0.01, 0.0);
		}
		mechanism = new Mechanism2d(1, 3);
		MechanismRoot2d root = mechanism.getRoot("Elevator", 0, 0);
		ligament = new MechanismLigament2d("Elevator", 0, 90);
		root.append(ligament);
		SmartDashboard.putData("Mechanisms/Elevator", mechanism);
		TelemetryManager.getInstance().addSendable(this);
		setDefaultCommand(stop());
	}

	@Override
	public void periodic() {
		lastReadHeight = ElevatorConstants.rotationsToHeight(
			leftMotor.getPosition().getValueAsDouble()) 
			+ ElevatorConstants.END_EFFECTOR_HEIGHT;
		leftMotor.setControl(request);
		if (Robot.isReal()) {
			ligament.setLength(lastReadHeight - ElevatorConstants.END_EFFECTOR_HEIGHT);
		}
	}

	@Override
	public void simulationPeriodic() {
		TalonFXSimState simState = leftMotor.getSimState();
		simState.setSupplyVoltage(RobotController.getBatteryVoltage());

		double motorVoltage = simState.getMotorVoltageMeasure().in(Units.Volts);
		elevatorSim.setInput(motorVoltage);
		elevatorSim.update(Constants.DT);

		double mechanismPositionRot = ElevatorConstants.heightToRotations(elevatorSim.getPositionMeters());
		double mechanismVelocityRotPerSec = ElevatorConstants.heightToRotations(elevatorSim.getVelocityMetersPerSecond());

		simState.setRawRotorPosition(mechanismPositionRot);
		simState.setRotorVelocity(mechanismVelocityRotPerSec);

		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

		ligament.setLength(elevatorSim.getPositionMeters());
	}

	private void setRequest(ControlRequest request) {
		this.request = request;
	}

	public Command moveToScoringHeight(ElevatorConstants.Heights height) {
		return moveToTarget(height.height).withName("Moving to height: " + height.name());
	}

	public Command moveToTarget(double targetHeight) {
		return runOnce(() -> setRequest(
			new MotionMagicDutyCycle(
				ElevatorConstants.heightToRotations(
					targetHeight - ElevatorConstants.END_EFFECTOR_HEIGHT))))
			.andThen(
				Commands.waitUntil(() -> isNearTarget(targetHeight)))
			.withName("Moving to height: " + targetHeight);
	}

	public Command stop() {
		return runOnce(() -> setRequest(new NeutralOut())).withName("Stopped");
	}

	public boolean isNearTarget(double targetHeight) {
		return MathUtil.isNear(
			lastReadHeight,
			targetHeight,
			ElevatorConstants.EPSILON);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty(
			"Height",
			() -> lastReadHeight,
			null);

		builder.addDoubleProperty(
            "Left/Volts",
            () -> leftMotor
                .getMotorVoltage()
                .getValue()
                .in(Units.Volts),
            null);

		builder.addDoubleProperty(
            "Left/Stator Current",
            () -> leftMotor
                .getStatorCurrent()
                .getValue()
                .in(Units.Amps),
            null);

		builder.addDoubleProperty(
            "Left/Temperature Celsius",
            () -> leftMotor
                .getDeviceTemp()
                .getValue()
                .in(Units.Celsius),
            null);

		builder.addDoubleProperty(
            "Left/Supply Current",
            () -> leftMotor
                .getSupplyCurrent()
                .getValue()
                .in(Units.Amps),
            null);

		builder.addDoubleProperty(
            "Left/Temperature Celsius",
            () -> leftMotor
                .getDeviceTemp()
                .getValue()
                .in(Units.Celsius),
            null);

		builder.addDoubleProperty(
			"Right/Volts",
			() -> rightMotor
				.getMotorVoltage()
				.getValue()
				.in(Units.Volts),
			null);

		builder.addDoubleProperty(
			"Right/Stator Current",
			() -> rightMotor
				.getStatorCurrent()
				.getValue()
				.in(Units.Amps),
			null);

		builder.addDoubleProperty(
			"Right/Temperature Celsius",
			() -> rightMotor
				.getDeviceTemp()
				.getValue()
				.in(Units.Celsius),
			null);

		builder.addDoubleProperty(
			"Right/Supply Current",
			() -> rightMotor
				.getSupplyCurrent()
				.getValue()
				.in(Units.Amps),
			null);

		builder.addDoubleProperty(
			"Right/Temperature Celsius",
			() -> rightMotor
				.getDeviceTemp()
				.getValue()
				.in(Units.Celsius),
			null);
	}
}