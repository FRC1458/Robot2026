package frc.robot.lib.subsystembases.servo;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.lib.subsystembases.SubsystemConstants;
import frc.robot.lib.util.CanDevice;

public abstract class ServoSubsystemConstants extends SubsystemConstants {
    public CanDevice main;
    public Angle positionEpsilon;
    public AngularVelocity velocityEpsilon;
    public Time debouncePeriod;
    
    public ServoSubsystemConstants(
        CanDevice device, 
        Angle posEps, 
        AngularVelocity velEps, 
        Time debounce
    ) {
        this.main = device;
        this.positionEpsilon = posEps;
        this.velocityEpsilon = velEps;
        this.debouncePeriod = debounce;
    }
}
