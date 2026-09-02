package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.subsystem.RollerMotorSubsystem;

public class RightIndexer extends RollerMotorSubsystem {
    public RightIndexer() {
        super(new ITalonFX(new TalonFX(R_MOTOR_ID), "Indexer/Right"));

        ((ITalonFX) io).configure(R_CONFIG);

        setDefaultCommand(stop());
    }

    public Command index() {
        return roll(
            FEEDER_SPEED, RotationsPerSecond.of(10), RunMode.VOLTAGE).andThen(idle());
    } 

    public Command stop() {
        return roll(
            RotationsPerSecond.of(0), RotationsPerSecond.of(10), RunMode.VOLTAGE);
    }
}
