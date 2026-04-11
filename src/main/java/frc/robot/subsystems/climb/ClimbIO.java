
package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.TalonFXIO;

public  class ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;
        public String currentCommand = "";
        public String defaultCommand = "";
    }

    private final String name;
    private final TalonFXIO motorIO;
    private final ClimbIOInputsAutoLogged inputs;

    public ClimbIO(String name, TalonFX motor) {
        this.name = name;
        motorIO = new TalonFXIO(name + "/Motor", motor);
        inputs = new ClimbIOInputsAutoLogged();
    }

    public void updateInputs(double positionMeters, double velocityMetersPerSecond, Command currentCommand, Command defaultCommand) {
        motorIO.updateInputs();

        inputs.positionMeters = positionMeters;
        inputs.velocityMetersPerSecond = velocityMetersPerSecond;
        inputs.currentCommand = currentCommand != null ? currentCommand.getName() : "None";
        inputs.defaultCommand = defaultCommand != null ? defaultCommand.getName() : "None";
    }

    public void process() {
        motorIO.process();
        Logger.processInputs(name, inputs);
    }
}