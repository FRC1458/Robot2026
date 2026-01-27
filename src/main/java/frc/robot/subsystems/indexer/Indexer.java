package frc.robot.subsystems.indexer;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TelemetryManager;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class Indexer extends SubsystemBase {
    private ControlRequest request;

    private TalonFX motor;
    private LaserCan lc;

    /** boolean that's modified by checkForBall() */
    private boolean hasBall;

    /** setup, adding motor and laser */
    private Indexer() {
        super();
        motor = new TalonFX(IndexerConstants.MOTOR_ID);
        lc = new LaserCan(IndexerConstants.LASER_ID);

        /* new laser configs */
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
          }
    }

    @Override
    /* check for balls and makes sure motor is constantly running at desired speed */
    public void periodic() {
        motor.setControl(request);
        checkForBall();
    }

    /** type conversion/abstraction */
    public void setRequest(ControlRequest request) {
        this.request = request;
    }

    /** sets speed (duh) */
    public  Command setSpeed(double speed) {
        return runOnce(() -> setRequest(new VelocityVoltage(speed)));
    }
    
    /** moves motor to speed if sense ball */
    public Command activateIndexer() {
        return Commands.either (
            setSpeed(IndexerConstants.ActiveSpeed),
            setSpeed(IndexerConstants.InactiveSpeed),
            this::hasBall
        );
    }

    /** turn motor down to zero */
    public Command deactivateIndexer() {
        return setSpeed(IndexerConstants.InactiveSpeed);
    }

    /** command to sense distance from camera; used to sense if bol */
    private double getDistanceMm() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        } else {
            return -1;
        }
    }

    /** setup formatting for boolean hasBall so it can be used in activateIndexer().either */
    private boolean hasBall() {
        return hasBall;
    }

    /** command that modifies hasBall, uses getDistanceMm() */
    private void checkForBall() {
        double x = getDistanceMm();
        if (x == IndexerConstants.LaserCan_DefaultMeasurement) {
            hasBall = false;
        } else {
            hasBall = true;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Has Ball", () -> hasBall, null);
        TelemetryManager.makeSendableTalonFX("Indexer motor", motor, builder);
    }
}