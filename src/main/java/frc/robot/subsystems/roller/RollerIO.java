package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.TalonFXIO;

public class RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public double velocityRPS = 0.0;
        public String currentCommand = "";
        public String defaultCommand = "";
    }

    private final String name;
    private final TalonFXIO motorIO;
    private final RollerIOInputsAutoLogged inputs;

    public RollerIO(String name, TalonFX motor) {
        this.name = name;
        motorIO = new TalonFXIO(name + "/Motor", motor);
        inputs = new RollerIOInputsAutoLogged();
    }

    public void updateInputs(double velocityRPS, Command currentCommand, Command defaultCommand) {
        motorIO.updateInputs();

        inputs.velocityRPS = velocityRPS;
        inputs.currentCommand = currentCommand != null ? currentCommand.getName() : "None";
        inputs.defaultCommand = defaultCommand != null ? defaultCommand.getName() : "None";
    }

    public void process() {
        motorIO.process();
        Logger.processInputs(name, inputs);
    }
}