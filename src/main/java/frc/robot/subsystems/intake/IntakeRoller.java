package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.IMotor;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.subsystem.RollerMotorSubsystem;

public class IntakeRoller extends RollerMotorSubsystem {
    public IntakeRoller(IMotor io) {
        super(new ITalonFX(new TalonFX(ROLLER_ID), "Intake/Roller"));

        ((ITalonFX) io).configure(ROLLER_CONFIG);

        setDefaultCommand(stop());
    }

    public Command intake() {
        return roll(
            ROLLER_SPEED, RotationsPerSecond.of(10), RunMode.VOLTAGE).andThen(idle());
    } 

    public Command outtake() {
        return roll(
            OUTTAKE_SPEED, RotationsPerSecond.of(10), RunMode.VOLTAGE).andThen(idle());
    } 

    public Command stop() {
        return roll(
            RotationsPerSecond.of(0), RotationsPerSecond.of(10), RunMode.VOLTAGE);
    }
}
