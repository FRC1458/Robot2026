package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

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

		request = new NeutralOut();
	}

	@Override
	public void periodic() {
		lastReadHeight = ElevatorConstants.rotationsToHeight(
			leftMotor.getPosition().getValueAsDouble()) 
			+ ElevatorConstants.END_EFFECTOR_HEIGHT;
		leftMotor.setControl(request);
	}

	@Override
	public void simulationPeriodic() {
		elevatorSim.setInput(motorSim.getAngularVelocityRadPerSec() * RobotController.getBatteryVoltage());
		elevatorSim.update(0.020);
		elevatorSim.setState(elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
		RoboRioSim.setVInVoltage(
        	BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
	}

	private void setRequest(ControlRequest request) {
		this.request = request;
	}

	public Command moveToScoringHeight(ElevatorConstants.Heights height) {
		return moveToTarget(height.height);
	}

	public Command moveToTarget(double targetHeight) {
		return Commands.runOnce(() -> setRequest(
			new MotionMagicDutyCycle(
				ElevatorConstants.heightToRotations(
					targetHeight - ElevatorConstants.END_EFFECTOR_HEIGHT))))
			.until(() -> isNearTarget(targetHeight)).andThen(stop());
	}

	public Command stop() {
		return Commands.runOnce(() -> setRequest(new NeutralOut()));
	}

	public boolean isNearTarget(double targetHeight) {
		return MathUtil.isNear(
			lastReadHeight,
			targetHeight,
			ElevatorConstants.EPSILON);
	}
}