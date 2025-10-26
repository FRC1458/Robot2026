package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX mLeftShooterMotor;
    private TalonFX mRightShooterMotor;

    public ShooterSubsystem() {
        mLeftShooterMotor = new TalonFX(Constants.Shooter.kShooterLeftMotorID); // ask for motor id's please
        mRightShooterMotor = new TalonFX(Constants.Shooter.kShooterRightMotorID); // this guy is the leader, other Fellow is the follower

        mLeftShooterMotor.setControl(new Follower(mRightShooterMotor.getDeviceID(), true));
        mRightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
        mLeftShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double speed) {
        mRightShooterMotor.set(speed);
    }

    public void stop() {
        setSpeed(0);
    }

    public void spin() {
        setSpeed(-0.05);
    }

    public void spinFast() {
        setSpeed(-0.15);
    }

    public void reverse() {
        setSpeed(0.05);
    }

}
