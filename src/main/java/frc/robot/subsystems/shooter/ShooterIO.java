package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.TalonFXIO;

public  class ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topWheelVelocityRPS = 0.0;
        public double bottomWheelVelocityRPS = 0.0;
        public String currentCommand = "";
        public String defaultCommand = "";
    }

    private final String name;
    private final TalonFXIO topMotorIO;
    private final TalonFXIO bottomMotorIO;
    private final ShooterIOInputsAutoLogged inputs;

    public ShooterIO(String name, TalonFX topMotor, TalonFX bottomMotor) {
        this.name = name;
        topMotorIO = new TalonFXIO(name + "/TopMotor", topMotor);
        bottomMotorIO = new TalonFXIO(name + "/BottomMotor", bottomMotor);
        inputs = new ShooterIOInputsAutoLogged();
    }

    public void updateInputs(double topWheelVelocityRPS, double bottomWheelVelocityRPS, Command currentCommand, Command defaultCommand) {
        topMotorIO.updateInputs();
        bottomMotorIO.updateInputs();

        inputs.topWheelVelocityRPS = topWheelVelocityRPS;
        inputs.bottomWheelVelocityRPS = bottomWheelVelocityRPS;
        inputs.currentCommand = currentCommand != null ? currentCommand.getName() : "None";
        inputs.defaultCommand = defaultCommand != null ? defaultCommand.getName() : "None";
    }

    public void process() {
        topMotorIO.process();
        bottomMotorIO.process();
        Logger.processInputs(name, inputs);
    }
}