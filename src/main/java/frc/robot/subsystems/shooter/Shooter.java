package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
	private double lastReadSpeedTop;
    private double lastReadSpeedBottom;
	private ControlRequest topRequest = new NeutralOut();
	private ControlRequest bottomRequest = new NeutralOut();

    private FlywheelSim topSim;
    private FlywheelSim bottomSim;

    private Shooter() {
        super();

		bottomMotor = new TalonFX(ShooterConstants.Motors.BOTTOM.id);
		bottomMotor.getConfigurator().apply(ShooterConstants.getConfig());
		bottomMotor.setNeutralMode(NeutralModeValue.Brake);

        topMotor = new TalonFX(ShooterConstants.Motors.TOP.id);
		topMotor.getConfigurator().apply(ShooterConstants.getConfig());
		topMotor.setNeutralMode(NeutralModeValue.Brake);
        
        if (Robot.isSimulation()) {
            topSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(1),
                    0.000489000861,
                    1
                ), DCMotor.getKrakenX60(1), 0.0);
            bottomSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX60(1),
                    0.000489000861,
                    1
                ), DCMotor.getKrakenX60(1), 0.0);
        }
		TelemetryManager.getInstance().addSendable(this);
    }

    @Override
    public void periodic() {
        // Read inputs
        lastReadSpeedTop = topMotor.getVelocity().getValueAsDouble();
        lastReadSpeedBottom = bottomMotor.getVelocity().getValueAsDouble();
        topMotor.setControl(topRequest);
        bottomMotor.setControl(bottomRequest);
    }

    @Override
    public void simulationPeriodic() {
		topMotor.getSimState()
            .setRotorVelocity(topSim.getAngularVelocityRPM() / 60.0);
		topMotor.getSimState().setSupplyVoltage(12.0);
        topSim.setInput(topMotor.getMotorVoltage().getValueAsDouble());
        topSim.update(0.020);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(topSim.getCurrentDrawAmps()));
        
		bottomMotor.getSimState()
            .setRotorVelocity(bottomSim.getAngularVelocityRPM() / 60.0);
		bottomMotor.getSimState().setSupplyVoltage(12.0);
        bottomSim.setInput(bottomMotor.getMotorVoltage().getValueAsDouble());
        bottomSim.update(0.020);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(bottomSim.getCurrentDrawAmps()));
    }

    /** Replaces the request */
    private void setTopRequest(ControlRequest request) {
        this.topRequest = request;
    }    
    
    /** Replaces the request */
    private void setBottomRequest(ControlRequest request) {
        this.bottomRequest = request;
    }

    /** Stops the shooter */
    public Command stop() {
        return runOnce(
            () -> {
                setTopRequest(new NeutralOut());
                setBottomRequest(new NeutralOut());
            }
        ).withName("Stopped");
    }

    public Command topShoot(double targetSpeed) {
        return runOnce(() -> setTopRequest(
            new VelocityVoltage(targetSpeed))
        ).withName("Top Shooting");
    }

    public Command bottomShoot(double targetSpeed) {
        return runOnce(() -> setBottomRequest(
            new VelocityVoltage(targetSpeed))
        ).withName("Bottom Shooting");
    }

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);

        builder.addDoubleProperty(
            "/TopSpeed", 
            () -> lastReadSpeedTop, 
            null);
		TelemetryManager.makeSendableTalonFX("/Top", topMotor, builder);

        builder.addDoubleProperty(
            "/BottomSpeed", 
            () -> lastReadSpeedBottom, 
            null);
		TelemetryManager.makeSendableTalonFX("/Bottom", bottomMotor, builder);

	}
}