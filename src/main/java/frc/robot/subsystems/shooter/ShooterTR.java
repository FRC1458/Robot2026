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

public class ShooterTR extends RollerMotorSubsystem {
    public ShooterTR(IMotor io) {
        super(new ITalonFX(new TalonFX(TR_ID), "Shooter/TR"));

        ((ITalonFX) io).configure(TR_CONFIG);

        setDefaultCommand(stop());
    }

    public Command shoot(DoubleSupplier distance) {
        return runOnce(() -> io.setRunMode(RunMode.VOLTAGE)).andThen(run(() -> {
            double rps = VEL_MAP.get(distance.getAsDouble()) - SPIN_MAP.get(distance.getAsDouble());
            
            io.setVelocity(RotationsPerSecond.of(rps));
        }));
    }

    public Command stop() {
        return roll(
            RotationsPerSecond.of(0), RotationsPerSecond.of(10), RunMode.VOLTAGE);
    }
}
