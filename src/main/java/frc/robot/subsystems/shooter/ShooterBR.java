package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.IMotor;
import frc.robot.lib.io.IMotor.RunMode;
import frc.robot.lib.io.ITalonFX;
import frc.robot.lib.subsystem.RollerMotorSubsystem;

public class ShooterBR extends RollerMotorSubsystem {
    public ShooterBR(IMotor io) {
        super(new ITalonFX(new TalonFX(BR_ID), "Shooter/BR"));

        ((ITalonFX) io).configure(BR_CONFIG);

        setDefaultCommand(stop());
    }

    public Command shoot(DoubleSupplier distance) {
        return runOnce(() -> io.setRunMode(RunMode.VOLTAGE)).andThen(run(() -> {
            double rps = VEL_MAP.get(distance.getAsDouble());
            
            io.setVelocity(RotationsPerSecond.of(rps));
        }));
    }

    public Command stop() {
        return roll(
            RotationsPerSecond.of(0), RotationsPerSecond.of(10), RunMode.VOLTAGE);
    }
}
