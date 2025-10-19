/*
package frc.robot.subsystems;

import java.time.Period;
import java.util.zip.Checksum;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Loops.ILooper;
import frc.robot.Loops.Loop;
import frc.robot.subsystems.SwerveDrive.PeriodicIO;
//both shooter and intake for coral
public class CoralShooter extends Subsystem {

	private static CoralShooter mInstance;
	
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	public static CoralShooter getInstance() {
		if (mInstance == null) {
		mInstance = new CoralShooter();
		}
		return mInstance;
	}

	private class PeriodicIO {
		double speed = 0.0;
		ShooterState state = ShooterState.INTAKE;
		boolean isShooting = false;
	} 
	
	private enum ShooterState {
		INTAKE,
		SHOOT,
		STOP
	}

	private TalonFX mLeftShooterMotor;
	private TalonFX mRightShooterMotor;

	private CoralShooter() {
		//super("Shooter");
		mLeftShooterMotor = new TalonFX(Constants.CoralShooter.kShooterLeftMotorId);
		mRightShooterMotor = new TalonFX(Constants.CoralShooter.kShooterRightMotorId); //LEADER
		mRightShooterMotor.setControl(new Follower(mLeftShooterMotor.getDeviceID(), true));
		mLeftShooterMotor.setNeutralMode(NeutralModeValue.Brake);
		mRightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
	}


	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
///* dc.2.10.25, commented out to compile so that we can merge GIT
				switch (mPeriodicIO.state) {
					case INTAKE:
						if (Laser.inRangeIntake()) {
							spin();
						} else {
							stop();
						}
						break;
					case SHOOT:
						if (Laser.inRangeShooter()) {
							spinFast();
						} else {
							intake();
						}
						break;
					default:
						System.err.println("coral shooter state corruption happened?");
						break;
				}

			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

	@Override
	public void writePeriodicOutputs() {
		mLeftShooterMotor.set(mPeriodicIO.speed);
	}

	@Override
	public void outputTelemetry() {

	}



	public void intake() {
		mPeriodicIO.state = ShooterState.INTAKE;
	}

	public void shoot() {
		mPeriodicIO.state = ShooterState.SHOOT;
	}

	public void spin() {
		mPeriodicIO.speed = Constants.CoralShooter.kShooterIntakeSpeed; //Constants.Shooter.kShooterSpeed;
	}

	public void spinFast() {
		mPeriodicIO.speed = Constants.CoralShooter.kShooterShootSpeed;
	}
	
	@Override
	public void stop() {
		mPeriodicIO.speed = 0.0;
	}
  
}
*/