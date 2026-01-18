package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.ControlRequest;
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

public class TopShooter extends SubsystemBase {
    private static TopShooter ShooterInstance;
	public static TopShooter getInstance() {
		if (ShooterInstance == null) {
			ShooterInstance = new TopShooter();
		}
		return ShooterInstance;
	}

    private final TalonFX topMotor;

    private final LaserCan shooterLaser;

    private final Debouncer shooterDebouncer;

    private boolean inRangeShooter;

	private double lastReadSpeed;
	private ControlRequest request = new NeutralOut();

    private TopShooter() {
        super();

        topMotor = new TalonFX(ShooterConstants.Motors.TOP.id);

        shooterLaser = new LaserCan(ShooterConstants.Lasers.FRONT.id);

		topMotor.getConfigurator().apply(ShooterConstants.getConfig());
		topMotor.setNeutralMode(NeutralModeValue.Brake);
        
        shooterDebouncer = new Debouncer(0.06, DebounceType.kBoth);

        inRangeShooter = false;

		TelemetryManager.getInstance().addSendable(this);
    }

    @Override
    public void periodic() {
        // Read inputs
        lastReadSpeed = topMotor.getVelocity().getValueAsDouble();
        inRangeShooter = shooterDebouncer.calculate(getShooterLaser());
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

	public Command spinUp()
	{
        ShooterConstants.SHOOT_SPEED_TOP = 25 + ShooterConstants.SHOOT_SPEED_INCREMENT;
        return runOnce(() -> setRequest(request)
        ).andThen(
            stop()
        ).withName("Increase Top Motor Speed");
	}

    public Command spinDown()
    {
        ShooterConstants.SHOOT_SPEED_TOP = 25 - ShooterConstants.SHOOT_SPEED_INCREMENT;
        return runOnce(() -> setRequest(request)
        ).andThen(
            stop()
        ).withName("Lower Top Motor Speed");
    }

    /* Gets whether the shooter laser detects the "fuel" (balls) */
    private boolean getShooterLaser() {
        return getMeasurementShooter() < 100;
    }

    public Command topShoot() {
        return runOnce(() -> setRequest(
            new VelocityVoltage(ShooterConstants.SHOOT_SPEED_TOP))
        ).andThen(
            Commands.waitUntil(() -> !inRangeShooter), 
            Commands.waitSeconds(0.15),
            stop()
        ).withName("Top Shooting");
    }

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
        builder.addDoubleProperty(
            "/Speed", 
            () -> lastReadSpeed, 
            null);
		TelemetryManager.makeSendableTalonFX("/Top", topMotor, builder);
	}
}