package frc.robot.subsystems.indexer;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TelemetryManager;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

// TODO (ethan): only activate if shooter ready
// TODO (ethan): ask tommy setControl(request)
public class Indexer extends SubsystemBase {
    /** getInstance of indexer */
    private static Indexer instance;
    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }
    private ControlRequest request;

    private TalonFX motor;
    private TalonFXSimState motorSim;
    private LaserCan lc, lcTwo;

    /** boolean that's modified by checkForBall() */
    private boolean hasBall;

    private boolean shooterReady = false;
    // COMMENTED OUT /** boolean that's modified by checkForBallTwo() */
    // private boolean hasBallTwo;

    /** boolean that's modified by shooter */

    /** setup, adding motor and laser */
    private Indexer() {
        super();
        motor = new TalonFX(IndexerConstants.MOTOR_ID);
        motorSim = motor.getSimState();
        lc = new LaserCan(IndexerConstants.LASER_ID);
        lcTwo = new LaserCan(IndexerConstants.LASER_ID_2);

        /* new laser configs */
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
        try {
            lcTwo.setRangingMode(LaserCan.RangingMode.SHORT);
            lcTwo.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lcTwo.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    @Override
    /* check for balls and makes sure motor is constantly running at desired speed */
    public void periodic() {
        motor.setControl(request);
        
        checkForBall();
        // checkForBallTwo();
        if (shooterReady = true) {
            activateIndexer();
        }
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

    /** command to sense distance from LaserCAN; used to sense if bol */
    private double getDistanceMm() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm;
        } else {
            return -1;
        }
    }
    // COMMENTED OUT /** command to sense if bol going into shooter */
    // private double getDistanceMmTwo() {
    //     LaserCan.Measurement measurement = lc.getMeasurement();
    //     if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
    //         return measurement.distance_mm;
    //     } else {
    //         return -1;
    //     }
    // }

    /** setup formatting for boolean hasBall so it can be used in activateIndexer().either */
    private boolean hasBall() {
        return hasBall;
    }
    // COMMENTED OUT /** setup formatting for boolean hasBall so that it can be used to tell if shooter ready */
    // private boolean hasBallTwo() {
    //     return hasBallTwo;
    // }

    /** command that modifies hasBall, uses getDistanceMm() */
    private void checkForBall() {
        double x = getDistanceMm();
        if (x >= IndexerConstants.LaserCan_DefaultMeasurement) {
            hasBall = false;
        } else if (x != -1) { /* (x != -1) is checking that the camera isn't just returning an error as ball sensed */
            hasBall = true;
        }
    }
    // COMMENTED OUT //** command that checks if bol is boutta be shot */
    // private void checkForBallTwo() {
    //     double x = getDistanceMmTwo();
    //     if (x >= IndexerConstants.LaserCan_DefaultMeasurement) {
    //         hasBallTwo = false;
    //     } else if (x != -1) { /* (x != -1) is checking that the camera isn't just returning an error as ball sensed */
    //         hasBallTwo = true;
    //     }
    // }
    
    @Override
    // TODO: AdvantageKit!
    /** ????????? */
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Has Ball", () -> hasBall, null);
        TelemetryManager.makeSendableTalonFX("Indexer motor", motor, builder);
    }
}