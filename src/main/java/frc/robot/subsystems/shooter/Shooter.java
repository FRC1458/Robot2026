package frc.robot.subsystems.shooter;

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

public class Shooter extends SubsystemBase {
    private static Shooter ShooterInstance;
	public static Shooter getInstance() {
		if (ShooterInstance == null) {
			ShooterInstance = new Shooter();
		}
		return ShooterInstance;
	}

    private final TalonFX leftMotor;
	private final TalonFX rightMotor;

    private final LaserCan shooterLaser;

    private final Debouncer shooterDebouncer;

    private boolean inRangeShooter; 

	private double lastReadSpeed;
	private ControlRequest request = new NeutralOut();

    private Shooter() {
        super();

        leftMotor = new TalonFX(ShooterConstants.Motors.LEFT.id);
		rightMotor = new TalonFX(ShooterConstants.Motors.RIGHT.id);

        shooterLaser = new LaserCan(ShooterConstants.Lasers.FRONT.id);

		leftMotor.getConfigurator().apply(ShooterConstants.getConfig());
		rightMotor.getConfigurator().apply(ShooterConstants.getConfig());
		leftMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setControl(
			new Follower(leftMotor.getDeviceID(), true));
        
        shooterDebouncer = new Debouncer(0.06, DebounceType.kBoth);

        inRangeShooter = false;

		TelemetryManager.getInstance().addSendable(this);
    }

    @Override
    public void periodic() {
        // Read inputs
        lastReadSpeed = leftMotor.getVelocity().getValueAsDouble();
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
    private double getMeasurementShooter() {
        var measurement = shooterLaser.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm;
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }

    /** Gets whether the intake laser detects a coral */
    private boolean getShooterLaser() {
        return getMeasurementShooter() < 100;
    }

    public Command shoot() {
        return runOnce(() -> setRequest(
            new VelocityVoltage(ShooterConstants.SHOOT_SPEED))
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