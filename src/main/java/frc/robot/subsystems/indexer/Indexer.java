package frc.robot.subsystems.indexer;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Indexer extends SubsystemBase{
    private ControlRequest request;

    private TalonFX motor;
    private SomeBrand laser; // fix later

    private boolean ballz = false;

    private Indexer() {
        motor = new TalonFX(IndexerConstants.MOTOR_ID);
    }

    private void IndexerSensor() {
        laser = new SomeBrand(IndexerConstants.LASER_ID);
    }

    @Override
    public void periodic() {
        motor.setControl(request);
        laser.setControl(request);
    }

    public void setRequest(ControlRequest request) {
        this.request = request;
    }

    public Command setSpeed(double speed) {
        return runOnce(() -> setRequest(new VelocityVoltage(speed)));
    }
    
    public Command activateIndexer() {
        return setSpeed(IndexerConstants.ActiveSpeed);
    }

    public Command deactivateIndexer() {
        return setSpeed(IndexerConstants.InactiveSpeed);
    }

    @Override

    public void setRequest2(ControlRequest request) {
        this.request = request;
    }

    public Command checkBalsl() {
    return runOnce(() -> {hasBall? = laser.get(); /* whatever laser detection is */});
    }

    public Command 


}