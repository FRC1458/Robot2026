package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.TelemetryManager;

public class Shooter extends SubsystemBase {
    private static Shooter shooterLeftInstance;
    private static Shooter shooterRightInstance;
	public static Shooter getLeftInstance() {
		if (shooterLeftInstance == null) {
			shooterLeftInstance = new Shooter(true);
		}
		return shooterLeftInstance;
	}

    
	public static Shooter getRightInstance() {
		if (shooterRightInstance == null) {
			shooterRightInstance = new Shooter(false);
		}
		return shooterRightInstance;
	}
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
	private double lastReadSpeedTop;
    private double lastReadSpeedBottom;
	private ControlRequest topRequest = new NeutralOut();
	private ControlRequest bottomRequest = new NeutralOut();

    private FlywheelSim topSim;
    private FlywheelSim bottomSim;

    private Shooter(boolean left) {
        super();

        int bottomID;
        int topID;

        if (left) {
            bottomID = ShooterConstants.Motors.BOTTOMLEFT.id;
            topID = ShooterConstants.Motors.TOPLEFT.id;
        }
        else {
            bottomID = ShooterConstants.Motors.BOTTOMRIGHT.id;
            topID = ShooterConstants.Motors.TOPRIGHT.id;
        }

		bottomMotor = new TalonFX(bottomID);
		bottomMotor.getConfigurator().apply(ShooterConstants.getConfig());
		bottomMotor.setNeutralMode(NeutralModeValue.Coast);

        topMotor = new TalonFX(topID);
		topMotor.getConfigurator().apply(ShooterConstants.getConfig());
		topMotor.setNeutralMode(NeutralModeValue.Coast);
        
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
        setDefaultCommand(stop());
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
        topMotor.getSimState().setSupplyVoltage(12);
        bottomMotor.getSimState().setSupplyVoltage(12);

        topSim.setInput(topMotor.getSimState().getMotorVoltage());
        topSim.update(0.020);
		topMotor.getSimState()
            .setRotorVelocity(topSim.getAngularVelocityRPM() / 60.0);
        topMotor.getSimState().addRotorPosition(topSim.getAngularVelocityRPM() / 60.0 * 0.020);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(topSim.getCurrentDrawAmps()));
        
        bottomSim.setInput(bottomMotor.getSimState().getMotorVoltage());
        bottomSim.update(0.020);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(bottomSim.getCurrentDrawAmps()));
		bottomMotor.getSimState()
            .setRotorVelocity(topSim.getAngularVelocityRPM() / 60.0);
        bottomMotor.getSimState().addRotorPosition(topSim.getAngularVelocityRPM() / 60.0 * 0.020);

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
                setTopRequest(new CoastOut());
                setBottomRequest(new CoastOut());
            }
        ).withName("Stopped");
    }

    public Command shoot(double topSpeed, double bottomSpeed) {
        return runOnce(() -> {
            setTopRequest(new VelocityVoltage(topSpeed));
            setBottomRequest(new VelocityVoltage(bottomSpeed));
        }).andThen(Commands.repeatingSequence(Commands.none())).withName("Shooting");
    }

    public Command shoot() {
        return shoot(512, -512);
    }

    @Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);

        builder.addDoubleProperty(
            "TopSpeed", 
            () -> lastReadSpeedTop, 
            null);
		TelemetryManager.makeSendableTalonFX("Top", topMotor, builder);

        builder.addDoubleProperty(
            "BottomSpeed", 
            () -> lastReadSpeedBottom, 
            null);
		TelemetryManager.makeSendableTalonFX("Bottom", bottomMotor, builder);
	}
}