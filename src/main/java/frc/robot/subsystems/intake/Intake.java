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

    private final TalonFX WheelMotor;
    private final TalonFX BarMotor;

    //private final Debouncer intakeDebouncer;

    private ControlRequest wheelRequest = new NeutralOut();
    private ControlRequest barRequest = new NeutralOut();

    //private boolean inRangeIntake;

    private Intake() {
        super();
        WheelMotor = new TalonFX(IntakeConstants.Motors.WHEEL.id);
        BarMotor = new TalonFX(IntakeConstants.Motors.BAR.id);
        WheelMotor.getConfigurator().apply(IntakeConstants.getWheelConfig());
		BarMotor.getConfigurator().apply(IntakeConstants.getBarConfig());
        WheelMotor.setNeutralMode(NeutralModeValue.Brake);
		BarMotor.setNeutralMode(NeutralModeValue.Brake); //?
		
        TelemetryManager.getInstance().addSendable(this);
        //setDefaultCommand();

    }


    @Override
    public void periodic(){
        //

        BarMotor.setControl(barRequest);
        WheelMotor.setControl(wheelRequest);
    }
    
    public Command stopWheel() {
        return runOnce(() -> setRequestWheel(new NeutralOut())).withName("Stopped");
    }

    public Command stopBar() {
        return runOnce(() -> setRequestBar(new NeutralOut())).withName("Stopped"); // should be voltage for upright bar?
    }

    private void setRequestWheel(ControlRequest request) {
        this.wheelRequest = request;
    }

    private void setRequestBar(ControlRequest request) {
        this.barRequest = request;
    }

    public Command setWheelIntaking() {
        return setWheelSpeed(IntakeConstants.INTAKE_SPEED);
    }

    public Command setWheelStop() {
        return runOnce(() -> setRequestWheel(new NeutralOut())); 
    }

    public Command setWheelSpeed(double speed) {
        return runOnce(() -> setRequestWheel (
            new VelocityVoltage(speed))
        ).withName("");
    }


    //---------BAR-----------


    public Command setBarDown() {
        return setBarPosition(IntakeConstants.BAR_POSITION_DOWN);
    }

    public Command setBarUp() {
        return setBarPosition(IntakeConstants.BAR_POSITION_UP);
    }
    
    public Command setBarPosition(double position) {
        double checkedPos = MathUtil.clamp(position, IntakeConstants.BAR_POS_MIN, IntakeConstants.BAR_POS_MAX);
        return runOnce(() -> setRequestBar(
            new PositionVoltage(checkedPos))
        ).withName("");
    }


}