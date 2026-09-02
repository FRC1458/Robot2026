package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.IMotor;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.subsystem.RollerMotorSubsystem;

public class Roller extends RollerMotorSubsystem {
    public Roller(IMotor io) {
        super(new ITalonFX(new TalonFX(ROLLER_ID), "Roller"));

        ((ITalonFX) io).configure(ROLLER_CONFIG);

        setDefaultCommand(stop());
    }

    public Command index() {
        return roll(
            ROLLER_SPEED, RotationsPerSecond.of(10), RunMode.VOLTAGE).andThen(idle());
    }

    public Command stop() {
        return roll(
            RotationsPerSecond.of(0), RotationsPerSecond.of(10), RunMode.VOLTAGE);
    }
}
