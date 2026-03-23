package frc.robot.lib.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.lib.io.Inputs;

@AutoLog
public class MotorInputs extends Inputs {
    public MutAngularVelocity velocity = 
        RotationsPerSecond.mutable(0.0);
    public MutAngle position = 
        Rotations.mutable(0.0);
    public MutVoltage appliedVoltage = 
        Volts.mutable(0.0);
    public MutCurrent statorCurrent = 
        Amps.mutable(0.0);
    public MutCurrent supplyCurrent = 
        Amps.mutable(0.0);
    public MutAngle rawRotorPosition = 
        Rotations.mutable(0.0);
    public MutTemperature temperature = 
        Celsius.mutable(0.0);
}
