package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.subsystem.HomingMotorSubsystem;

public class IntakePivot extends HomingMotorSubsystem {
    public IntakePivot() {
        super(new ITalonFX(new TalonFX(PIVOT_ID), "Intake/Pivot"));

        ((ITalonFX) io).configure(PIVOT_CONFIG);
    }

    public Command lower() {
        return home(
            PIVOT_POS_MIN, 
            PIVOT_EPSILON, 
            RunMode.VOLTAGE_TRAPEZOIDAL);
    }

    public Command raise() {
        return home(
            PIVOT_POS_UP, 
            PIVOT_EPSILON, 
            RunMode.VOLTAGE_TRAPEZOIDAL);
    }

    public Command shake() {
        return Commands.repeatingSequence(
            runOnce(() -> io.setPosition(PIVOT_POS_MID)),
            Commands.waitSeconds(0.3),
            runOnce(() -> io.setPosition(PIVOT_POS_MID)),
            Commands.waitSeconds(0.3));
    }

    public Command stop() {
        return runOnce(() -> io.setNeutral());
    }
}
