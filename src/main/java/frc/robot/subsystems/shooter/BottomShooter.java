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

public class BottomShooter extends SubsystemBase {
    private static BottomShooter ShooterInstance;
	public static BottomShooter getInstance() {
		if (ShooterInstance == null) {
			ShooterInstance = new BottomShooter();
		}
		return ShooterInstance;
	}

	private final TalonFX bottomMotor;

    private final LaserCan shooterLaser;

    private final Debouncer shooterDebouncer;

    private boolean inRangeShooter;

	private double lastReadSpeed;
	private ControlRequest request = new NeutralOut();

    private BottomShooter() {
        super();

		bottomMotor = new TalonFX(ShooterConstants.Motors.BOTTOM.id);

        shooterLaser = new LaserCan(ShooterConstants.Lasers.FRONT.id);

		bottomMotor.getConfigurator().apply(ShooterConstants.getConfig());
		bottomMotor.setNeutralMode(NeutralModeValue.Brake);
        
        shooterDebouncer = new Debouncer(0.06, DebounceType.kBoth);

        inRangeShooter = false;

		TelemetryManager.getInstance().addSendable(this);
    }

    @Override
    public void periodic() {
        // Read inputs
        lastReadSpeed = bottomMotor.getVelocity().getValueAsDouble();
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
        ShooterConstants.SHOOT_SPEED_BOTTOM = 25 + ShooterConstants.SHOOT_SPEED_INCREMENT;
        return runOnce(() -> setRequest(request)
        ).andThen(
            stop()
        ).withName("Increase Bottom Motor Speed");
	}

    public Command spinDown()
    {
        ShooterConstants.SHOOT_SPEED_BOTTOM = 25 - ShooterConstants.SHOOT_SPEED_INCREMENT;
        return runOnce(() -> setRequest(request)
        ).andThen(
            stop()
        ).withName("Lower Bottom Motor Speed");
    }

    /** Gets whether the shooter laser detects the "fuel" (balls) */
    private boolean getShooterLaser() {
        return getMeasurementShooter() < 100;
    }

    public Command BottomShoot() {
        return runOnce(() -> setRequest(
            new VelocityVoltage(ShooterConstants.SHOOT_SPEED_BOTTOM))
        ).andThen(
            Commands.waitUntil(() -> !inRangeShooter), 
            Commands.waitSeconds(0.15),
            stop()
        ).withName("Bottom Shooting");
    }

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
        builder.addDoubleProperty(
            "/Speed", 
            () -> lastReadSpeed, 
            null);
		TelemetryManager.makeSendableTalonFX("/Bottom", bottomMotor, builder);
	}
}