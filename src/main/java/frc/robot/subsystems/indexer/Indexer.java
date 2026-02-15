package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.TelemetryManager;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

// TODO (ethan): only activate if shooter ready
// TODO (ethan): ask tommy setControl(request)
public class Indexer extends SubsystemBase {
    /** getInstance of indexer */
    private static Indexer leftInstance;
    public static Indexer getLeftInstance() {
        if (leftInstance == null) {
            leftInstance = new Indexer(true);
        }
        return leftInstance;
    }
    private static Indexer rightInstance;
    public static Indexer getRightInstance() {
        if (rightInstance == null) {
            rightInstance = new Indexer(false);
        }
        return rightInstance;
    }
    private ControlRequest request;

    private TalonFX motor;
    // private LaserCan lc;

    /** boolean that's modified by checkForBall() */
    // private boolean hasBall;

    private FlywheelSim wheelSim;

    // private boolean shooterReady = false;
    // COMMENTED OUT /** boolean that's modified by checkForBallTwo() */
    // private boolean hasBallTwo;

    /** boolean that's modified by shooter */

    /** setup, adding motor and laser */
    private Indexer(boolean isLeft) {
        super();
        setName("Indexer " + (isLeft ? "Left" : "Right"));
        motor = new TalonFX(isLeft ? L_MOTOR_ID : R_MOTOR_ID);
        motor.getConfigurator().apply(getConfig());
        // lc = new LaserCan(isLeft ? L_LASER_ID : R_LASER_ID);
        // lcTwo = new LaserCan(LASER_ID_2);

        /* new laser configs */
        // if (Robot.isReal()) {
        // for (int i = 0; i < 20; i++) {
        //     try {
        //         lc.setRangingMode(LaserCan.RangingMode.SHORT);
        //         lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        //         lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        //         break;
        //     } catch (ConfigurationFailedException e) {
        //         System.out.println("Configuration failed! " + e);
        //     }
        // }}

        if (Robot.isSimulation()) {
            wheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                    DCMotor.getKrakenX44(1),
                    0.000189000861,
                    2), 
                DCMotor.getKrakenX44(1), 
                0.0);
        }
        // try {
        //     lcTwo.setRangingMode(LaserCan.RangingMode.SHORT);
        //     lcTwo.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        //     lcTwo.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        // } catch (ConfigurationFailedException e) {
        //     System.out.println("Configuration failed! " + e);
        // }

        TelemetryManager.getInstance().addSendable(this);
        setDefaultCommand(deactivateIndexer());
    }

    @Override
    /* check for balls and makes sure motor is constantly running at desired speed */
    public void periodic() {
        motor.setControl(request);
        
        // checkForBall();
        // checkForBallTwo();
        // if (shooterReady == true) {
        //     activateIndexer();
        // }
    }

    @Override
    public void simulationPeriodic() {
        

        motor.getSimState().setSupplyVoltage(12);

        wheelSim.setInput(motor.getSimState().getMotorVoltage());
        wheelSim.update(0.020);
		motor.getSimState()
            .setRotorVelocity(wheelSim.getAngularVelocityRPM() / 60.0);
        motor.getSimState().addRotorPosition(wheelSim.getAngularVelocityRPM() / 60.0 * 0.020);
    }

    /** type conversion/abstraction */
    public void setRequest(ControlRequest request) {
        this.request = request;
    }

    /** sets speed (duh) */
    public Command setSpeed(double speed) {
        return runOnce(() -> setRequest(new VelocityVoltage(speed)));
    }
    
    /** moves motor to speed if sense ball */
    // public Command loadIndexer() {
    //     return Commands.either(
    //         deactivateIndexer(),
    //         activateIndexer(),
    //         this::hasBall
    //     ).repeatedly();
    // }

    public Command activateIndexer() {
        return setSpeed(ROLLING_SPEED);
    }

    /** turn motor down to zero */
    public Command deactivateIndexer() {
        return setSpeed(0);
    }

    /** command to sense distance from LaserCAN; used to sense if bol */
    // private double getDistanceMm() {
    //     LaserCan.Measurement measurement = lc.getMeasurement();
    //     if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
    //         return measurement.distance_mm;
    //     } else {
    //         return Double.POSITIVE_INFINITY;
    //     }
    // }

    /** command that modifies hasBall, uses getDistanceMm() */
    // private void checkForBall() {
    //     double x = getDistanceMm();
    //     if (x <= MAXIMUM_LASER_DIST) {
    //         hasBall = false;
    //     }
    // }

    // /** setup formatting for boolean hasBall so it can be used in activateIndexer().either */
    // private boolean hasBall() {
    //     return hasBall;
    // }
    // COMMENTED OUT /** setup formatting for boolean hasBall so that it can be used to tell if shooter ready */
    // private boolean hasBallTwo() {
    //     return hasBallTwo;
    // }
    // COMMENTED OUT /** command to sense if bol going into shooter */
    // private double getDistanceMmTwo() {
    //     LaserCan.Measurement measurement = lc.getMeasurement();
    //     if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
    //         return measurement.distance_mm;
    //     } else {
    //         return -1;
    //     }
    // }
    // COMMENTED OUT //** command that checks if bol is boutta be shot */
    // private void checkForBallTwo() {
    //     double x = getDistanceMmTwo();
    //     if (x >= LaserCan_DefaultMeasurement) {
    //         hasBallTwo = false;
    //     } else if (x != -1) { /* (x != -1) is checking that the camera isn't just returning an error as ball sensed */
    //         hasBallTwo = true;
    //     }
    // }
    
    // TODO: AdvantageKit!
    /** ????????? */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // builder.addBooleanProperty("Has Ball", () -> hasBall, null);
        TelemetryManager.makeSendableTalonFX("Indexer Motor", motor, builder);
    }
}