package frc.robot.subsystems.coralshooter;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

    private final Debouncer intakeDebouncer;
    private final Debouncer shooterDebouncer;

    private boolean inRangeIntake;
    private boolean inRangeShooter; 

	private double lastReadSpeed;
	private ControlRequest request = new NeutralOut();

    private CoralShooter() {
        super();

        leftMotor = new TalonFX(CoralShooterConstants.Motors.LEFT.id);
		rightMotor = new TalonFX(CoralShooterConstants.Motors.RIGHT.id);

        intakeLaser = new LaserCan(CoralShooterConstants.Lasers.BACK.id);
        shooterLaser = new LaserCan(CoralShooterConstants.Lasers.FRONT.id);

		leftMotor.getConfigurator().apply(CoralShooterConstants.getConfig());
		rightMotor.getConfigurator().apply(CoralShooterConstants.getConfig());
		leftMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setControl(
			new Follower(leftMotor.getDeviceID(), true));
        
        intakeDebouncer = new Debouncer(0.06, DebounceType.kBoth);
        shooterDebouncer = new Debouncer(0.06, DebounceType.kBoth);

        inRangeIntake = false;
        inRangeShooter = false;

		TelemetryManager.getInstance().addSendable(this);
		setDefaultCommand(listenAndIntake());
    }

    @Override
    public void periodic() {
        // Read inputs
        lastReadSpeed = leftMotor.getVelocity().getValueAsDouble();
        inRangeIntake = intakeDebouncer.calculate(getIntakeLaser());
        inRangeShooter = shooterDebouncer.calculate(getShooterLaser());
        leftMotor.setControl(request);
    }

    /** Replaces the request */
    private void setRequest(ControlRequest request) {
        this.request = request;
    }

    /** Stops the shooter */
    public Command stop() {
        return runOnce(() -> setRequest(new NeutralOut())).withName("Stopped");
    }

    /** Gets the intake laser measurement */
    private double getMeasurementIntake() {
        var measurement = intakeLaser.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm;
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }

    /** Gets the intake laser measurement */
    private double getMeasurementShooter() {
        var measurement = shooterLaser.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm;
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }

    /** Gets whether the intake laser detects a coral */
    private boolean getIntakeLaser() {
        return getMeasurementIntake() < 100;
    }

    /** Gets whether the intake laser detects a coral */
    private boolean getShooterLaser() {
        return getMeasurementShooter() < 100;
    }

    /** Returns whether a coral might be obstructing the elevator */
    public boolean isCoralObstructingElevator() {
        return inRangeIntake;
    }

    public Command listenAndIntake() {
        return Commands.repeatingSequence(
            stop(),
            Commands.waitUntil(() -> inRangeIntake),
            intake()
        ).withName("Listen, Intake");
    }

    public Command intake() {
        return runOnce(() -> setRequest(
            new VelocityVoltage(CoralShooterConstants.INTAKE_SPEED))
        ).andThen( 
            Commands.waitUntil(() -> !inRangeIntake)
        ).withName("Intaking");
    }

    public Command shoot() {
        return runOnce(() -> setRequest(
            new VelocityVoltage(CoralShooterConstants.SHOOT_SPEED))
        ).andThen(
            Commands.waitUntil(() -> !inRangeShooter), 
            Commands.waitSeconds(0.15),
            stop()
        ).withName("Shooting");
    }

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
        builder.addDoubleProperty(
            "/Speed", 
            () -> lastReadSpeed, 
            null);
		TelemetryManager.makeSendableTalonFX("/Left", leftMotor, builder);
		TelemetryManager.makeSendableTalonFX("/Right", rightMotor, builder);
	}
}