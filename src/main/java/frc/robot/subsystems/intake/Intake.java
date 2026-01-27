package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TelemetryManager;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Intake extends SubsystemBase {
    private static Intake intakeInstance;

    public static Intake getInstance() {
        if (intakeInstance == null) {
            intakeInstance = new Intake(); 
        }
        return intakeInstance;
    }

    private final TalonFX wheelMotor;
    private final TalonFX barMotor;
    private double wheelSpeed;
    private double barPosition;

    private ControlRequest wheelRequest = new NeutralOut();
    private ControlRequest barRequest = new NeutralOut();

    private Intake() {
        super();
        wheelMotor = new TalonFX(IntakeConstants.Motors.WHEEL.id);
        barMotor = new TalonFX(IntakeConstants.Motors.BAR.id);
        wheelMotor.getConfigurator().apply(IntakeConstants.getWheelConfig());
		barMotor.getConfigurator().apply(IntakeConstants.getBarConfig());
        wheelMotor.setNeutralMode(NeutralModeValue.Brake);
		barMotor.setNeutralMode(NeutralModeValue.Brake); //?

        TelemetryManager.getInstance().addSendable(this);
    }


    @Override
    public void periodic(){
        wheelSpeed = wheelMotor.getVelocity().getValueAsDouble();
        barPosition = barMotor.getPosition().getValueAsDouble();
        barMotor.setControl(barRequest);
        wheelMotor.setControl(wheelRequest);
    }
    
    //---------------stop----------------

    /**
     * sends neutral request to wheel 
     */
     
    public Command stopWheel() {
        return runOnce(() -> setRequestWheel(new NeutralOut())).withName("Stopped");
    }

    /**
     * sends position voltage request that attempts to keep bar in place
     */
    public Command stopBar() {
        return runOnce(() -> setRequestBar(new PositionVoltage(barPosition))).withName("Stopped"); //needs testing
    }

    //----------------set request---------------
    private void setRequestWheel(ControlRequest request) {
        this.wheelRequest = request;
    }

    private void setRequestBar(ControlRequest request) {
        this.barRequest = request;
    }

    //----------------wheel----------------

    /**
     * sets wheel speed to intake speed
     */

    public Command setWheelIntaking() {
        return setWheelSpeed(IntakeConstants.INTAKE_SPEED);
    }

    /**
     * sets the wheel speed (in rotations per second)
     * @return
     */

    public Command setWheelSpeed(double speed) {
        return runOnce(() -> setRequestWheel (
            new VelocityVoltage(speed))
        ).withName("wheel speed set " + (speed));
    }


    //---------bar-----------

    /**
     * sets request to the bar down voltage
     * @return
     */
    public Command setBarDown() {
        return setBarPosition(IntakeConstants.BAR_POSITION_DOWN);
    }

    /**
     * sets request to the bar up voltage
     * @return
     */
    public Command setBarUp() {
        return setBarPosition(IntakeConstants.BAR_POSITION_UP);
    }
    
    /**
     * sets position request within limits and then requests that position (in rotations)
     * @param position
     * @return
     */
    public Command setBarPosition(double position) {
        double checkedPos = MathUtil.clamp(position, IntakeConstants.BAR_POS_MIN, IntakeConstants.BAR_POS_MAX);
        return runOnce(() -> setRequestBar(
            new PositionVoltage(checkedPos))
        ).withName("bar pos set" + (checkedPos));
    }
}