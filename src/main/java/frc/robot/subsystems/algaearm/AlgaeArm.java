/*package frc.robot.subsystems;

import java.lang.constant.DirectMethodHandleDesc;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Loops.ILooper;
import frc.robot.Loops.Loop;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.DigitalSensor;

public class AlgaeShooter extends Subsystem {

    private static AlgaeShooter mInstance = null;
    private PeriodicIO mPeriodicIO;

    // private static final double kPivotCLRampRate = 0.5;
    // private static final double kCLRampRate = 0.5;

    public static AlgaeShooter getInstance() {
        if (mInstance == null) {
            mInstance = new AlgaeShooter();
        }
        return mInstance;
    }

    private TalonFX mPivotMotor;


    private MotionMagicVoltage m_request;
    private boolean mSafeStop = true;
    private DigitalInput sensor;



    private AlgaeShooter() {
        mPeriodicIO = new PeriodicIO();

        mPivotMotor = new TalonFX(Constants.AlgaeShooter.kAlgaePivotMotorId); // MASTER
        

        var talonFXConfigs = new TalonFXConfiguration();

        sensor = new DigitalInput(3);

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = Constants.AlgaeShooter.kS; // Add 0.0 V output to overcome static friction
        slot0Configs.kV = Constants.AlgaeShooter.kV; // A velocity target of 1 rps results in 0.0 V output
        slot0Configs.kP = Constants.AlgaeShooter.kP; // An error of 1 rotation results in 0.4 V output
        slot0Configs.kI = Constants.AlgaeShooter.kI; // no output for integrated error
        slot0Configs.kD = Constants.AlgaeShooter.kD; // A velocity of 1 rps results in 0.0 V output

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.AlgaeShooter.kCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = Constants.AlgaeShooter.kAcceleration; // Target acceleration of 240 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = Constants.AlgaeShooter.kJerk;

        mPivotMotor.getConfigurator().apply(talonFXConfigs);

//        mPivotMotor.setControl(new DutyCycleOut(mPivotMotor.getDutyCycle().getValue()));//dc.3.15.25, why is this here? it could cause bugs

        m_request = new MotionMagicVoltage(0);
    }

    private static class PeriodicIO {
        double pivot_target = 0.0;
        double des_shooter_speed = 0.0;
        String state = "Resting";
        double mCurrentPos = 0.0;//current encoder reading 
    }


	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
        
			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

    @Override
    public void readPeriodicInputs() {
        //if ()
        mPeriodicIO.mCurrentPos = mPivotMotor.getPosition().getValueAsDouble(); //update pivot current position
    }


    @Override
    public void writePeriodicOutputs() {
            goToTarget();
    }

    @Override
    public void stop() {
        mPeriodicIO.pivot_target = mPivotMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Position/Target", mPeriodicIO.pivot_target);
    }

    public void resetRot(double pos) {
        mPivotMotor.setPosition(pos);
    }


    public void runPivotRaw(double speed) {
        mPivotMotor.set(speed);
    }

    public synchronized void setTarget(String targ) {
        mSafeStop = false;
        String prevString = targ;
        mPeriodicIO.state = targ;
        switch (targ) {
            case "Resting":
                mPeriodicIO.pivot_target = Constants.AlgaeShooter.kRestingPosition;
                break;
            case "Intake":
                mPeriodicIO.pivot_target = Constants.AlgaeShooter.kIntakePosition;
                break;
            case "Barge":
                mPeriodicIO.pivot_target = Constants.AlgaeShooter.kBargePosition;
                break;
            case "Processor":
                mPeriodicIO.pivot_target = Constants.AlgaeShooter.kProcessorPosition;
                break;
            case "Ground":
                mPeriodicIO.pivot_target = Constants.AlgaeShooter.kGroundPosition;
                break;
            default:
                mPeriodicIO.state = prevString;
                break;
        }
        
    }


    private void goToTarget() {
        mPivotMotor.setControl(m_request.withPosition(mPeriodicIO.pivot_target));
    }

    public synchronized boolean isAtTarget() {
        return Math.abs(mPeriodicIO.mCurrentPos - mPeriodicIO.pivot_target) < 0.5; //TODO: tune magic number in constants
    }

    public void shoot() {
        mPeriodicIO.des_shooter_speed = Constants.AlgaeShooter.kAlgaeShooterSpeed;
    }

    public void intake() {
        mPeriodicIO.des_shooter_speed = -Constants.AlgaeShooter.kAlgaeShooterSpeed;
    }

    public void stopAlgaeShooter() {
        mPeriodicIO.des_shooter_speed = 0.0;
    }
}
*/