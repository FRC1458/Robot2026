package frc.robot.subsystems.algaearm;

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

public class AlgaeArm extends SubsystemBase {
	private static AlgaeArm AlgaeArmInstance;
	public static AlgaeArm getInstance() {
		if (AlgaeArmInstance == null) {
			AlgaeArmInstance = new AlgaeArm();
		}
		return AlgaeArmInstance;
	}

	private final TalonFX pivotMotor;

	private ControlRequest request;

    private double lastReadHeight;

	//private DCMotorSim motorSim;
	//private AlgaeArmSim algaeArmSim; we need to make sim
	private Mechanism2d mechanism;
	private MechanismLigament2d ligament;

	private AlgaeArm() {
		pivotMotor = new TalonFX(AlgaeArmConstants.Motors.PIVOT_MOTOR.id);
		pivotMotor.getConfigurator().apply(AlgaeArmConstants.getConfig());
		pivotMotor.setNeutralMode(NeutralModeValue.Brake);

		mechanism = new Mechanism2d(1, 3);
		MechanismRoot2d root = mechanism.getRoot("AlgaeArm", 0, 0);
		ligament = new MechanismLigament2d("AlgaeArm", 0, 90);
		root.append(ligament);
		SmartDashboard.putData("Mechanisms/AlgaeArm", mechanism);
		TelemetryManager.getInstance().addSendable(this);
		setDefaultCommand(stop());
	}

	@Override
	public void periodic() {

		lastReadHeight = AlgaeArmConstants.rotationsToHeight(
			pivotMotor.getPosition().getValueAsDouble()) 
			+ AlgaeArmConstants.END_EFFECTOR_HEIGHT;
		pivotMotor.setControl(request);
		if (Robot.isReal()) {
			ligament.setLength(lastReadHeight - AlgaeArmConstants.END_EFFECTOR_HEIGHT);
		}
	}
/* code from elevator to be fitted with AlgaeArm
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
	}*/

	private void setRequest(ControlRequest request) {
		this.request = request;
	}

	public Command moveToScoringHeight(AlgaeArmConstants.Heights height) { // TODO: change setpoint
		return moveToTarget(height.height).withName("Moving to height: " + height.name());
	}

	public Command moveToTarget(double targetHeight) {
		return runOnce(() -> setRequest(
			new MotionMagicDutyCycle(
				AlgaeArmConstants.heightToRotations(
					targetHeight - AlgaeArmConstants.END_EFFECTOR_HEIGHT))))
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
			AlgaeArmConstants.EPSILON);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty(
			"Height",
			() -> lastReadHeight,
			null);

		builder.addDoubleProperty(
            "PivotMotor/Volts",
            () -> pivotMotor
                .getMotorVoltage()
                .getValue()
                .in(Units.Volts),
            null);

		builder.addDoubleProperty(
            "PivotMotor/Stator Current",
            () -> pivotMotor
                .getStatorCurrent()
                .getValue()
                .in(Units.Amps),
            null);

		builder.addDoubleProperty(
            "PivotMotor/Temperature Celsius",
            () -> pivotMotor
                .getDeviceTemp()
                .getValue()
                .in(Units.Celsius),
            null);

		builder.addDoubleProperty(
            "PivotMotor/Supply Current",
            () -> pivotMotor
                .getSupplyCurrent()
                .getValue()
                .in(Units.Amps),
            null);

		builder.addDoubleProperty(
            "PivotMotor/Temperature Celsius",
            () -> pivotMotor
                .getDeviceTemp()
                .getValue()
                .in(Units.Celsius),
            null);
	}
}