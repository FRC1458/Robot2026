package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.TalonFXIO;

public class IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double wheelVelocityRPS = 0.0;
        public double barPositionDeg = 0.0;
        public String currentCommand = "";
        public String defaultCommand = "";
    }

    private final String name;
    private final TalonFXIO wheelMotorIO;
    private final TalonFXIO barMotorIO;
    private final IntakeIOInputsAutoLogged inputs;

    public IntakeIO(String name, TalonFX wheelMotor, TalonFX barMotor) {
        this.name = name;
        wheelMotorIO = new TalonFXIO(name + "/WheelMotor", wheelMotor);
        barMotorIO = new TalonFXIO(name + "/BarMotor", barMotor);
        inputs = new IntakeIOInputsAutoLogged();
    }

    public void updateInputs(double wheelVelocityRPS, double barPositionDeg, Command currentCommand, Command defaultCommand) {
        wheelMotorIO.updateInputs();
        barMotorIO.updateInputs();

        inputs.wheelVelocityRPS = wheelVelocityRPS;
        inputs.barPositionDeg = barPositionDeg;
        inputs.currentCommand = currentCommand != null ? currentCommand.getName() : "None";
        inputs.defaultCommand = defaultCommand != null ? defaultCommand.getName() : "None";
    }

    public void process() {
        wheelMotorIO.process();
        barMotorIO.process();
        Logger.processInputs(name, inputs);
    }
}