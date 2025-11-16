package frc.robot.subsystems.coralshooter;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TelemetryManager;

public class CoralShooter extends SubsystemBase {
    private static CoralShooter coralShooterInstance;
	public static CoralShooter getInstance() {
		if (coralShooterInstance == null) {
			coralShooterInstance = new CoralShooter();
		}
		return coralShooterInstance;
	}

    private final TalonFX leftMotor;
	private final TalonFX rightMotor;

    private final LaserCan intakeLaser;
    private final LaserCan shooterLaser;

	private double lastReadSpeed;
	private ControlRequest request = new NeutralOut();

    private CoralShooter() {
        super();

        leftMotor = new TalonFX(CoralShooterConstants.Motors.LEFT.id);
		rightMotor = new TalonFX(CoralShooterConstants.Motors.RIGHT.id);

        intakeLaser = new LaserCan(CoralShooterConstants.Lasers.BACK.id);
        shooterLaser = new LaserCan(CoralShooterConstants.Lasers.FRONT.id);;

		leftMotor.getConfigurator().apply(CoralShooterConstants.getConfig());
		rightMotor.getConfigurator().apply(CoralShooterConstants.getConfig());
		leftMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setControl(
			new Follower(leftMotor.getDeviceID(), true));
        
		TelemetryManager.getInstance().addSendable(this);
		setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        lastReadSpeed = leftMotor.getVelocity().getValueAsDouble();
        leftMotor.setControl(request);
    }

    private void setRequest(ControlRequest request) {
        this.request = request;
    }

    public Command stop() {
        return runOnce(() -> setRequest(new NeutralOut())).withName("Stopped");
    }

    public double getMeasurementIntake() {
        return intakeLaser.getMeasurement().distance_mm;
    }

    public double getMeasurementShooter() {
        return shooterLaser.getMeasurement().distance_mm;
    }

    public boolean inRangeIntake() {
        return getMeasurementIntake() < 100;
    }

    public boolean inRangeShooter() {
        return getMeasurementShooter() < 100;
    }

    public boolean isCoralObstructingElevator() {
        return inRangeIntake();
    }

    public Command intake() {
        return runOnce(
            () -> setRequest(
                new VelocityVoltage(CoralShooterConstants.MAX_SPEED)))
            .andThen(Commands.waitUntil(() -> !inRangeIntake())).withName("Intaking");
    }

    public Command shoot() {
        return runOnce(() -> setRequest(
            new VelocityVoltage(CoralShooterConstants.MAX_SPEED)))
        .andThen(Commands.waitUntil(() -> !inRangeShooter())).withName("Shooting");
    }

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty(
			"Speed",
			() -> lastReadSpeed,
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