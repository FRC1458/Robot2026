package frc.robot.subsystems.indexer;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class Indexer extends SubsystemBase {
    private ControlRequest request;

    private TalonFX motor;
    private LaserCan lc;

    private double getDistanceMm() /* command to sense distance from camera */ {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        } else {
            return -1;
        }
    }

    private boolean hasBall; /* boolean that's modified by checkForBall() */

    private boolean hasBall() /* setup formatting for boolean hasBall so it can be used in activateIndexer().either */ {
        return hasBall;
    }

    private void checkForBall() /* command that modifies hasBall */ {
        double x = getDistanceMm();
        if (x == IndexerConstants.LaserCan_DefaultMeasurement) {
            boolean hasBall = false;
        } else {
            boolean hasBall = true;
        }
    }

    private Indexer() /* setup, adding motor and laser */ {
        motor = new TalonFX(IndexerConstants.MOTOR_ID);

        lc = new LaserCan(IndexerConstants.LASER_ID);
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          }
    }

    @Override
    public void periodic() /* check for balls and makes sure motor is constantly running at desired speed */ {
        motor.setControl(request);
        checkForBall();
    }

    public void setRequest(ControlRequest request) /* type conversion/abstraction    */ {
        this.request = request;
    }

    public  Command setSpeed(double speed) {
        return runOnce(() -> setRequest(new VelocityVoltage(speed)));
    }
    
    public Command activateIndexer() {
        return Commands.either (
            setSpeed(IndexerConstants.ActiveSpeed),
            setSpeed(IndexerConstants.InactiveSpeed),
            this::hasBall
        );
    }

    public Command deactivateIndexer() {
        return setSpeed(IndexerConstants.InactiveSpeed);
    }
}