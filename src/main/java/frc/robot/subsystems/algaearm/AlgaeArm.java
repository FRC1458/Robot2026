package frc.robot.subsystems.algaearm;

import static frc.robot.subsystems.algaearm.AlgaeArmConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.elevator.Elevator;

public class AlgaeArm extends SubsystemBase {
    private static AlgaeArm algaeArmInstance;
    public static AlgaeArm getInstance() {
        if (algaeArmInstance == null) {
            algaeArmInstance = new AlgaeArm();
        }
        return algaeArmInstance;
    }

    private ControlRequest request = new NeutralOut();
    private final TalonFX pivotMotor;
    private double currentAngle;
    
	private SingleJointedArmSim armSim;
	private MechanismLigament2d ligament;

    private AlgaeArm() {
        pivotMotor = new TalonFX(Motors.PIVOT_MOTOR.id);
        pivotMotor.getConfigurator().apply(getConfig());
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        if (Robot.isSimulation()) {
            armSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60Foc(1),
                GEAR_RATIO,
                0.49 * LENGTH * LENGTH,
                LENGTH,
                MIN_ANGLE - Constants.TAU * 1.0 / 24.0,
                Constants.TAU,
                true,
                Constants.TAU * 1.0 / 4.0);
        }
        
		ligament = new MechanismLigament2d("AlgaeArm", LENGTH, 90);
        Elevator.getInstance().getLigament().append(ligament);
        setDefaultCommand(stop());
		TelemetryManager.getInstance().addSendable(this);
    }

    @Override
    public void periodic() {
		// Read the angle from the motor encoder
        currentAngle = rotationsToAngle(
            pivotMotor.getPosition().getValueAsDouble());
        // updates the motor
        pivotMotor.setControl(request);
    }

    @Override
	public void simulationPeriodic() {
		// sets the voltages
		TalonFXSimState simState = pivotMotor.getSimState();
		simState.setSupplyVoltage(RobotController.getBatteryVoltage());

		// updates the arm sim
		double motorVoltage = simState.getMotorVoltageMeasure().in(Units.Volts);
		armSim.setInput(-motorVoltage);
		armSim.update(Constants.DT);

		// Gets the motor position and velocities from the arm
		double mechanismPositionRot = 
            angleToRotations(
                Constants.TAU * 1.0 / 4.0 - armSim.getAngleRads());
		double mechanismVelocityRotPerSec = 
            angleToRotations(armSim.getVelocityRadPerSec());

		// Puts them back in the motor sim
		simState.setRawRotorPosition(mechanismPositionRot);
		simState.setRotorVelocity(mechanismVelocityRotPerSec);

		// Updates the battery
		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

		// updates the widget
		ligament.setAngle(-armSim.getAngleRads() / Constants.TAU * 360);
	}
    
	/** Swaps the control request */
    private void setRequest(ControlRequest request) {
        this.request = request;
    }
    
	/** Moves to a preset angle */
    public Command moveToAngle(Angles angle) {
        return moveToAngle(angle.angle);//.withName(angle.name() + ":Moving to angle");
    }

    /** Moves to a raw angle */
    public Command moveToAngle(double targetAngle) {
        return defer(() -> {
            double targetRotations = angleToRotations(targetAngle);
			// Move motor to target
            return runOnce(() -> setRequest(new MotionMagicVoltage(targetRotations)))
                .andThen(
                    Commands.waitUntil(() -> isNearTarget(targetAngle)),
                    stop()); 
        }).withName(targetAngle + ":Moving to angle (raw)");
    }

    /** Stops the mechanism */
    public Command stop() {
        return runOnce(() -> setRequest(new PositionVoltage(
            pivotMotor.getPosition().getValue()))).withName("Stopped");
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
					state -> SignalLogger.writeString("SysIdArm_State", state.toString())
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

    /** Is the arm near its target */
    public boolean isNearTarget(double target) {
        return MathUtil.isNear(target, currentAngle, EPSILON);
    }

    /** Returns the current angle */
    public double getCurrentAngle() {
        return currentAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Arm Angle", this::getCurrentAngle, null);
        
        TelemetryManager.makeSendableTalonFX("Pivot Motor", pivotMotor, builder);
    }
}
