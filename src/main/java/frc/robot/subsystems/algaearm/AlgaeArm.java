package frc.robot.subsystems.algaearm;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {

    private static AlgaeArm algaeArmInstance;

	private ControlRequest request;

    public static AlgaeArm getInstance() {
        if (algaeArmInstance == null) {
            algaeArmInstance = new AlgaeArm();
        }
        return algaeArmInstance;
    }

    private final TalonFX pivotMotor;
    private double currentAngle;

    private AlgaeArm() {
        pivotMotor = new TalonFX(AlgaeArmConstants.Motors.PIVOT_MOTOR.id);
        pivotMotor.getConfigurator().apply(AlgaeArmConstants.getConfig());
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        currentAngle = AlgaeArmConstants.rotationsToAngle(pivotMotor.getPosition().getValueAsDouble());
		pivotMotor.setControl(request);
    }

    public Command moveToAngle(double targetAngle) {
		return defer(() -> {
			double targetRotations = AlgaeArmConstants.angleToRotations(
				MathUtil.clamp(targetAngle, AlgaeArmConstants.MIN_ANGLE, AlgaeArmConstants.MAX_ANGLE));

			return runOnce(() -> 
				setRequest(new MotionMagicVoltage(targetRotations)) // Updates motor
			).withName("Moving to angle: " + targetAngle);
		});
    }

	private void setRequest(ControlRequest request) {
		this.request = request;
	}

	/** Stops the AlgaeArm */
    public Command stop() {
        return runOnce(() -> setRequest(
			new PositionVoltage(pivotMotor.getPosition().getValue())))
		.withName("Stop Arm");
    }
	
    public double getCurrentAngle() {
        return currentAngle;
    }
}
