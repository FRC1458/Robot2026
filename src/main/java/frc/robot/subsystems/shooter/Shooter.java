package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.text.Format;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.FieldPoses;
import frc.robot.subsystems.shooter.ShooterConstants.Motors;

public class Shooter extends SubsystemBase {
    private static Shooter shooterLeftInstance;
    private static Shooter shooterRightInstance;

    public static Shooter getLeftInstance() {
        if (shooterLeftInstance == null) {
            shooterLeftInstance = new Shooter(true);
        }
        return shooterLeftInstance;
    }

    public static Shooter getRightInstance() {
        if (shooterRightInstance == null) {
            shooterRightInstance = new Shooter(false);
        }
        return shooterRightInstance;
    }

    private final TalonFX topMotor;
    private final TalonFX bottomMotor;
    private double lastReadSpeedTop;
    private double lastReadSpeedBottom;
    private ControlRequest topRequest = new NeutralOut();
    private ControlRequest bottomRequest = new NeutralOut();

    private FlywheelSim topSim;
    private FlywheelSim bottomSim;

    // private ShotCalculator shotCalculator = ShotCalculator.getInstance();

    private InterpolatingDoubleTreeMap spin = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap tree = new InterpolatingDoubleTreeMap();

    private ShooterIO io;

    private Shooter(boolean left) {
        super();

        setName(this.getClass().getSimpleName() + (left ? "Left" : "Right"));

        int bottomID;
        int topID;

        if (left) {
            bottomID = Motors.BOTTOMLEFT.id;
            topID = Motors.TOPLEFT.id;
        } else {
            bottomID = Motors.BOTTOMRIGHT.id;
            topID = Motors.TOPRIGHT.id;
        }

        var tconfig = getTopConfig();

        if (!left) {
            tconfig = tconfig.clone().withMotorOutput(
                    new MotorOutputConfigs()
                            .withInverted(InvertedValue.Clockwise_Positive));
        }

        var bconfig = getBottomConfig();

        if (!left) {
            bconfig = bconfig.clone().withMotorOutput(
                    new MotorOutputConfigs()
                            .withInverted(InvertedValue.Clockwise_Positive));
        }

        bottomMotor = new TalonFX(bottomID);
        bottomMotor.getConfigurator().apply(bconfig);
        bottomMotor.setNeutralMode(NeutralModeValue.Coast);

        topMotor = new TalonFX(topID);
        topMotor.getConfigurator().apply(tconfig);
        topMotor.setNeutralMode(NeutralModeValue.Coast);

        if (Robot.isSimulation()) {
            topSim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            0.000489000861,
                            1),
                    DCMotor.getKrakenX60(1), 0.0);
            bottomSim = new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            0.000489000861,
                            1),
                    DCMotor.getKrakenX60(1), 0.0);
        }

        io = new ShooterIO(getName(), topMotor, bottomMotor);

        // tree.put(1.24, 27.0);
        // tree.put(1.56, 28.0);
        // tree.put(1.752, 29.8);
        // tree.put(2.005, 32.0);
        // tree.put(2.77, 36.0);
        // tree.put(3.09, 44.0);
        // tree.put(2.45, 34.5);
        // tree.put(2.3, 33.3);
        // tree.put(1.87, 32.0);
        // tree.put(2.094, 32.6);
        // tree.put(2.91, 39.8);

        // tree.put(2.0, 30.0);
        // tree.put(1.5, 23.5);
        // tree.put(2.5, 44.0);
        // tree.put(3.5, 63.0);
        tree.put(1.5, 16.0);
        tree.put(2.0, 20.0);
        tree.put(2.5, 30.0);
        tree.put(3.5, 50.0);

        spin.put(1.5, 0.0);
        spin.put(2.0, 0.0);
        spin.put(2.5, 15.0);
        spin.put(3.5, 15.0);
    }

    @Override
    public void periodic() {
        // Read inputs
        lastReadSpeedTop = topMotor.getVelocity().getValueAsDouble();
        lastReadSpeedBottom = bottomMotor.getVelocity().getValueAsDouble();
        topMotor.setControl(topRequest);
        bottomMotor.setControl(bottomRequest);

        SmartDashboard.putNumber(
            "topShooterVel",
            tree.get(SmartDashboard.getNumber("toShooter", 1.5))
        );
        SmartDashboard.putNumber(
            "bottomShooterDel", 
            spin.get(SmartDashboard.getNumber("toShooter", 0))
        );

        io.updateInputs(lastReadSpeedTop, lastReadSpeedBottom, getCurrentCommand(), getDefaultCommand());
        io.process();
    }

    public double getTopSpeed() {
        return lastReadSpeedTop;
    }

    @Override
    public void simulationPeriodic() {
        topMotor.getSimState().setSupplyVoltage(12);
        bottomMotor.getSimState().setSupplyVoltage(12);

        topSim.setInput(topMotor.getSimState().getMotorVoltage());
        topSim.update(0.020);
        topMotor.getSimState()
                .setRotorVelocity(topSim.getAngularVelocityRPM() / 60.0);
        topMotor.getSimState().addRotorPosition(topSim.getAngularVelocityRPM() / 60.0 * 0.020);
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(topSim.getCurrentDrawAmps()));

        bottomSim.setInput(bottomMotor.getSimState().getMotorVoltage());
        bottomSim.update(0.020);
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(bottomSim.getCurrentDrawAmps()));
        bottomMotor.getSimState()
                .setRotorVelocity(topSim.getAngularVelocityRPM() / 60.0);
        bottomMotor.getSimState().addRotorPosition(topSim.getAngularVelocityRPM() / 60.0 * 0.020);

    }

    /** Replaces the request */
    private void setTopRequest(ControlRequest request) {
        this.topRequest = request;
    }

    /** Replaces the request */
    private void setBottomRequest(ControlRequest request) {
        this.bottomRequest = request;
    }

    /** Stops the shooter */
    public Command stop() {
        return runOnce(
                () -> {
                    setTopRequest(new CoastOut());
                    setBottomRequest(new CoastOut());
                }).withName("Stopped");
    }

    public void shoot(double topSpeed, double bottomSpeed) {
//        return runOnce(() -> {
            setTopRequest(new VelocityVoltage(topSpeed));
            setBottomRequest(new VelocityVoltage(bottomSpeed));
//        }).withName("Shooting");
    }

    public Command shoot(DoubleSupplier distance) {
        // System.out.println("Shooter/shoot()/distance: " + distance.getAsDouble()+ 
        //     ", top motor RPS: " + tree.get(distance.getAsDouble()) + 
        //     ", spin delta: " + spin.get(distance.getAsDouble()));
        return runOnce(()->{shoot(
            tree.get(distance.getAsDouble()),
            tree.get(distance.getAsDouble()) - spin.get(distance.getAsDouble()));
        }).withName("Shooting");
                // distance_to_shooter.get(distance.getAsDouble())
                //         .get(0, 0) - 15,
                // distance_to_shooter.get(distance.getAsDouble())
                //         .get(1, 0) + 15);
    }

    public Command shoot(DoubleSupplier topSpeed, DoubleSupplier bottomSpeed) {
        var topReq = new VelocityVoltage(0.0);
        var bottomReq = new VelocityVoltage(0.0);
        return runOnce(() -> {
            setTopRequest(topReq);
            setBottomRequest(bottomReq);
        }).andThen(
                run(() -> {
                    topReq.withVelocity(topSpeed.getAsDouble());
                    bottomReq.withVelocity(bottomSpeed.getAsDouble());
                })).withName("Shooting");
    }

    public Command shoot() {
        // return shoot(30, 30);
        
        return shoot(() -> Drive.getInstance().getPose().getTranslation()
                .getDistance(
                        Constants.FieldConstants.allianceCorrected(
                                FieldPoses.HUB.pose3d.getTranslation()).toTranslation2d()));
    }

    public void runVolts(Voltage voltage) {
        setBottomRequest(new VoltageOut(voltage));
    }

    public SysIdRoutine sysId() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, // Use default config
                        (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        this::runVolts,
                        null, // No log consumer, since data is recorded by AdvantageKit
                        this));
    }
}