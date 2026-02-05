package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import javax.lang.model.element.TypeElement;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.TelemetryManager;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;


public class Intake extends SubsystemBase {
    private static Intake intakeInstance;

    public static Intake getInstance() {
        if (intakeInstance == null) {
            intakeInstance = new Intake(); 
        }
        return intakeInstance;
    }

    private final TalonFX wheelMotor;
    private TalonFXSimState wheelMotorSim;
    private final TalonFX barMotor;
    private final TalonFXSimState barMotorSim;

    private double wheelSpeed;
    private double barPosition;

    private ControlRequest wheelRequest = new NeutralOut();
    private ControlRequest barRequest = new NeutralOut();

    private SingleJointedArmSim sim;
    /*
    private MechanismLigament2d ligament;   

    private final Mechanism2d mech2d = new Mechanism2d(20, 20);
    private final MechanismRoot2d mech2droot = mech2d.getRoot("Bar Root",  10, 1);
    */

    private Intake() {
        super();
        wheelMotor = new TalonFX(Motors.WHEEL.id);
        wheelMotorSim = wheelMotor.getSimState();
        barMotor = new TalonFX(Motors.BAR.id);
        barMotorSim = barMotor.getSimState();
        wheelMotor.getConfigurator().apply(getWheelConfig());
		barMotor.getConfigurator().apply(getBarConfig());
        wheelMotor.setNeutralMode(NeutralModeValue.Brake);
		barMotor.setNeutralMode(NeutralModeValue.Brake); //?

        // SmartDashboard.putData("123123", mech2d);
        // //TelemetryManager.getInstance().addSendable(this);
        // mech2droot.append(mech2dpivot);

        if (Robot.isSimulation()) {
            sim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getKrakenX60(1),1, 2),
            DCMotor.getKrakenX60(1),
            2.0,
            INTAKE_LENGTH,
            BAR_POS_MIN,
            BAR_POS_MAX,
            true,
            0.0,
            0.0, 0.0
            );
        }
        
        TelemetryManager.getInstance().addSendable(this);
            
    }

    
    
    @Override
    public void periodic(){
        wheelSpeed = wheelMotor.getVelocity().getValueAsDouble();
        barPosition = barMotor.getPosition().getValueAsDouble();
        barMotor.setControl(barRequest);
        wheelMotor.setControl(wheelRequest);
        //ligament.setAngle(barPosition);
    }
    
    //----------------set request---------------
    private void setRequestWheel(ControlRequest request) {
        this.wheelRequest = request;
    }

    private void setRequestBar(ControlRequest request) {
        this.barRequest = request;
    }


    public Command setSetpoint(double wheelSpeed, double barPosition) {
        return 
            setWheelSpeed(wheelSpeed)
            .andThen(setBarPosition(barPosition), 
            Commands.print("something happen"))
            .withName("Setpoint reached probably");
    }


    //---------bar-----------

    /**
     * sets request to the bar down voltage
     * @return
     */
    public Command setBarDown() {
        return setBarPosition(BAR_POSITION_DOWN);
    }

    /**
     * sets request to the bar up voltage
     * @return
     */
    public Command setBarUp() {
        return setBarPosition(BAR_POSITION_UP);
    }
    
    /**
     * sets position request within limits and then requests that position (in rotations)
     * @param position
     * @return
     */
    public Command setBarPosition(double position) {
        double checkedPos = MathUtil.clamp(position, BAR_POS_MIN, BAR_POS_MAX);
        return runOnce(() ->
            setRequestBar(
            new PositionVoltage(checkedPos))
        ).withName("bar pos set" + (checkedPos));
    }

    public Command setWheelSpeed(double speed) {
        return runOnce(() -> setRequestWheel(
            new VelocityVoltage(speed))
        ).withName("wheel speed set "+ (speed));
    }

    public Command waitUntilBarIsAtPosition(double target) {
        return Commands.waitUntil(() -> Math.abs(target - barPosition) < BAR_EPSILON);
    }
    
    public Command waitUntilWheelIsAtSpeed(double target) {
        return Commands.waitUntil(() -> Math.abs(target - wheelSpeed) < WHEEL_EPSILON);
    }
    @Override
    public void simulationPeriodic() {
        barMotor.getSimState().setSupplyVoltage(12.0);
        sim.setInput(barMotor.getSimState().getMotorVoltage());
        sim.update(0.020);
        barMotor.getSimState()
            .setRawRotorPosition(sim.getAngleRads() * (1 / Constants.TAU));
        
        
        
    }

    public void initSendable(SendableBuilder builder){ 
        super.initSendable(builder);
        builder.addDoubleProperty("Position", () -> barPosition, null);
        TelemetryManager.makeSendableTalonFX("Intake123", barMotor, builder);

    }


    public Command intake() {
        return setSetpoint(INTAKE_SPEED, BAR_POSITION_DOWN)
            .andThen(waitUntilBarIsAtPosition(BAR_POSITION_DOWN));
    }

    public Command outtake() {
        return setSetpoint(-INTAKE_SPEED, BAR_POSITION_DOWN)
            .andThen(waitUntilBarIsAtPosition(BAR_POSITION_DOWN));
    }

    public Command stow() {
        return setSetpoint(0.0, BAR_POSITION_UP)
            .andThen(waitUntilBarIsAtPosition(BAR_POSITION_UP));
    }
}