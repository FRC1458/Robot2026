package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;


public class Roller extends SubsystemBase {
    private static Roller rollerInstance;

    public static Roller getInstance() {
        if (rollerInstance == null) {
            rollerInstance = new Roller(); 
        }
        return rollerInstance;
    }

    private final TalonFX motor;
    private double speed;

    private ControlRequest request = new NeutralOut();

    private FlywheelSim sim;
    private RollerIO io;

    private Roller() {
        super();
        motor = new TalonFX(MOTOR_ID);
        motor.getConfigurator().apply(getConfig());
        motor.setNeutralMode(NeutralModeValue.Brake);

        if (Robot.isSimulation()) {
            sim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX44(1),
                    0.000189000861,
                    2), 
                DCMotor.getKrakenX44(1), 
                0.0);
        }

        io = new RollerIO(getName(), motor);
        // TelemetryManager.getInstance().addSendable(this);
    }

    @Override
    public void periodic(){
        speed = motor.getVelocity().getValueAsDouble();
        motor.setControl(request);
        io.updateInputs(speed, getCurrentCommand(), getDefaultCommand());
        io.process();
    }

    @Override
    public void simulationPeriodic() {
        motor.getSimState().setSupplyVoltage(12);
        sim.setInput(motor.getSimState().getMotorVoltage());
        sim.update(0.020);
		motor.getSimState()
            .setRotorVelocity(sim.getAngularVelocityRPM() / 60.0);
        motor.getSimState().addRotorPosition(sim.getAngularVelocityRPM() / 60.0 * 0.020);
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
    }

    private void setRequest(ControlRequest request) {
        this.request = request;
    }

    public Command setSpeed(double speed) {
        var req = new VelocityVoltage(speed);
        return runOnce(
            () -> setRequest(req)
        ).withName(speed + ":Speed");
    }

    public Command roll() {
        return setSpeed(ROLL_SPEED);
    }

    public Command stop() {
        return runOnce(() -> setRequest(new NeutralOut()))
            .withName("Stop");
    }

    // @Override
    // public void initSendable(SendableBuilder builder){ 
    //     super.initSendable(builder);
    //     builder.addDoubleProperty("Speed", () -> speed, null);
    //     TelemetryManager.makeSendableTalonFX("Roller Motor", motor, builder);
    // }


}