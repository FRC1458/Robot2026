package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import au.grapplerobotics.LaserCan;
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

    private final TalonFX topMotor;

    private final TalonFX bottomMotor;

    //private final LaserCan shooterLaser;

    private final Debouncer shooterDebouncer;

    private boolean inRangeShooter;

	private double lastReadSpeedTop;
    private double lastReadSpeedBottom;
	private ControlRequest request = new NeutralOut();

    private Shooter() {
        super();

		bottomMotor = new TalonFX(ShooterConstants.Motors.BOTTOM.id);

        //shooterLaser = new LaserCan(ShooterConstants.Lasers.FRONT.id);

		bottomMotor.getConfigurator().apply(ShooterConstants.getConfig());
		bottomMotor.setNeutralMode(NeutralModeValue.Brake);

        topMotor = new TalonFX(ShooterConstants.Motors.TOP.id);

		topMotor.getConfigurator().apply(ShooterConstants.getConfig());
		topMotor.setNeutralMode(NeutralModeValue.Brake);
        
        shooterDebouncer = new Debouncer(0.06, DebounceType.kBoth);

        //inRangeShooter = false;

		TelemetryManager.getInstance().addSendable(this);
    }

    @Override
    public void periodic() {
        // Read inputs
        lastReadSpeedTop = topMotor.getVelocity().getValueAsDouble();
        lastReadSpeedBottom = bottomMotor.getVelocity().getValueAsDouble();
        //inRangeShooter = shooterDebouncer.calculate(getShooterLaser());
    }

    /** Replaces the request */
    private void setRequest(ControlRequest request) {
        this.request = request;
    }

    /** Stops the shooter */
    public Command stop() {
        return runOnce(() -> setRequest(new NeutralOut())).withName("Stopped");
    }

    /** Gets the shooter laser measurement (Not needed as of now) */
    /*
    private double getMeasurementShooter() {
        var measurement = shooterLaser.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm;
        } else {
            return Double.POSITIVE_INFINITY;
        }
    }
    */

    /* Gets whether the shooter laser detects the "fuel" (balls) not needed right now
    private boolean getShooterLaser() {
        return getMeasurementShooter() < 100;
    }
    */

    public Command topShoot(int targetSpeed) {
        return runOnce(() -> setRequest(
            new VelocityVoltage(targetSpeed))
        ).andThen(
            Commands.waitUntil(() -> !inRangeShooter), 
            Commands.waitSeconds(0.15),
            stop()
        ).withName("Top Shooting");
    }

    public Command BottomShoot(int targetSpeed) {
        return runOnce(() -> setRequest(
            new VelocityVoltage(targetSpeed))
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
            () -> lastReadSpeedTop, 
            null);
		TelemetryManager.makeSendableTalonFX("/Top", topMotor, builder);

        builder.addDoubleProperty(
            "/Speed", 
            () -> lastReadSpeedBottom, 
            null);
		TelemetryManager.makeSendableTalonFX("/Bottom", bottomMotor, builder);

	}
}