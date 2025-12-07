package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.field.FieldLayout;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.ctre.CtreDriveConstants;
import frc.robot.subsystems.drive.commands.PIDToPoseCommand;
import frc.robot.subsystems.drive.ctre.CtreDrive;
import frc.robot.subsystems.drive.ctre.CtreDriveTelemetry;

public class Drive extends SubsystemBase {
	private static Drive driveInstance;
    public static Drive getInstance() {
        if (driveInstance == null) {
            driveInstance = new Drive();
        }
        return driveInstance;
    }

	private SwerveDriveState lastReadState;
    public static final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric();
	public SwerveRequest driveRequest = teleopRequest;

    private final CtreDrive drivetrain = CtreDriveConstants.createDrivetrain();    
	private final CtreDriveTelemetry telemetry = new CtreDriveTelemetry(MAX_SPEED);
    @SuppressWarnings("unused") 
    private Time lastPoseResetTime = BaseUnits.TimeUnit.of(0.0); // Citrus what are you doing

	private Drive() {
		lastReadState = drivetrain.getState();
		drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
			return driveRequest;
		}));

		drivetrain.getOdometryThread().setThreadPriority(31);
        TelemetryManager.getInstance().addStructPublisher("Mechanisms/Drive", Pose3d.struct, () -> new Pose3d(getPose()));
        TelemetryManager.getInstance().addSendable(this);
	}

    /** @return the ctre generated drivetrain */
	public CtreDrive getCtreDrive() {
		return drivetrain;
	}

	@Override
	public void periodic() {
		lastReadState = drivetrain.getState();
		outputTelemetry();
	}

	public void outputTelemetry() {
		telemetry.telemeterize(lastReadState);
        FieldLayout.field.setRobotPose(getPose());
	}

    /**
     * @return the current state
     */
	public SwerveDriveState getState() {
		return drivetrain.getState();
	}

    /**
     * @return the last read pose
     */
	public Pose2d getPose() {
		return lastReadState.Pose;
	}

    /** 
     * @return the chassis speeds, field relative
     */
    public ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(lastReadState.Speeds, lastReadState.Pose.getRotation());
    }
	
    /**
     * Switches the swerve request
     * <p>Please do not the new swerve request every 20 ms</p>
     */
    public void setSwerveRequest(SwerveRequest request) {
	 	driveRequest = request;
	}

    /**
     * @return the current swerve request
     */
    public SwerveRequest getSwerveRequest() {
		return driveRequest; 
	}

    /** Open loop during teleop mode */
    public Command teleopCommand() {
        return runOnce(() -> {
            teleopRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
            setSwerveRequest(teleopRequest);
        }).andThen(run(() -> {
            double xDesiredRaw = -Robot.controller.getLeftY();
            double yDesiredRaw = -Robot.controller.getLeftX();
            double rotDesiredRaw = -Robot.controller.getRightX();
            double xFancy = Util.applyJoystickDeadband(xDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);
            double yFancy = Util.applyJoystickDeadband(yDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);
            double rotFancy = Util.applyJoystickDeadband(rotDesiredRaw, Constants.Controllers.DRIVER_DEADBAND);

            SmartDashboard.putNumber("Sticks/vX", xDesiredRaw);
            SmartDashboard.putNumber("Sticks/vY", yDesiredRaw);
            SmartDashboard.putNumber("Sticks/vW", rotDesiredRaw);

            teleopRequest
                .withVelocityX(xFancy * MAX_SPEED)
                .withVelocityY(yFancy * MAX_SPEED)
                .withRotationalRate(rotFancy * MAX_ROTATION_SPEED);        
        }).handleInterrupt(() -> setSwerveRequest(new SwerveRequest.FieldCentric()))).withName("Teleop");
    }

    /** 
     * Auto aligns to the nearest reef face
     * @param left chooses the left or right face
     */
    public Command autoAlign(boolean left) {
        return defer(() -> {
            Pose2d pose;
            if (left) {
                pose = getPose().nearest(FieldLayout.ALIGN_POSES_LEFT);
            } else {
                pose = getPose().nearest(FieldLayout.ALIGN_POSES_RIGHT);
            }
            return new PIDToPoseCommand(pose).withName("Auto Align to " + pose);
        });
    }

    /** Adds a vision update */
	public void addVisionUpdate(Pose2d pose, Time timestamp) {
		getCtreDrive().addVisionMeasurement(pose, timestamp.in(Units.Seconds));
	}

    /** Adds a vision update with standard deviations */
	public void addVisionUpdate(Pose2d pose, Time timestamp, Matrix<N3, N1> stdDevs) {
		getCtreDrive().addVisionMeasurement(pose, timestamp.in(Units.Seconds), stdDevs);
	}

    /** Resets pose estimator to a pose */
	public void resetPose(Pose2d pose) {
		getCtreDrive().resetPose(pose);
		lastPoseResetTime =
            Units.Seconds.of(Utils.getCurrentTimeSeconds()).plus(POSE_RESET_PREVENTION_TIME);
	}

    /** A command that resets the pose */
	public Command resetPoseCommand(Pose2d pose) {
		return Commands.runOnce(() -> resetPose(pose));
	}

    /** Whether the pitch is stable */
	public boolean isPitchStable() {
		return drivetrain.getPigeon2().getAngularVelocityYDevice().getValue().abs(Units.DegreesPerSecond)
                < MAX_VELOCITY_STABLE
            && drivetrain.getPigeon2().getPitch().getValue().abs(BaseUnits.AngleUnit)
                < MAX_PITCH_STABLE;
	}

    /** Whether the roll is stable */
	public boolean isRollStable() {
		return drivetrain.getPigeon2().getAngularVelocityXDevice().getValue().abs(Units.DegreesPerSecond)
                < MAX_VELOCITY_STABLE
            && drivetrain.getPigeon2().getRoll().getValue().abs(BaseUnits.AngleUnit)
                < MAX_PITCH_STABLE;
	}

    /** Whether the robot is stable */
	public boolean isStable() {
		ChassisSpeeds speeds = getState().Speeds;
		return isPitchStable()
            && isRollStable()
            && Units.MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
                .lte(MAX_SPEED_SCORING_TRANSLATION)
            && Units.RadiansPerSecond.of(speeds.omegaRadiansPerSecond).lte(MAX_ROTATION_SPEED_SCORING);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
		builder.addDoubleProperty(
            "Pitch Velocity Degrees Per Second",
            () -> drivetrain
                .getPigeon2()
                .getAngularVelocityYDevice()
                .getValue()
                .in(Units.DegreesPerSecond),
            null);
		builder.addDoubleProperty(
            "Pitch Degrees",
            () -> drivetrain.getPigeon2().getPitch().getValue().in(Units.Degrees),
            null);

		builder.addDoubleProperty(
            "Roll Velocity Degrees Per Second",
            () -> drivetrain
                .getPigeon2()
                .getAngularVelocityXDevice()
                .getValue()
                .in(Units.DegreesPerSecond),
            null);
		builder.addDoubleProperty(
            "Roll Degrees",
            () -> drivetrain.getPigeon2().getRoll().getValue().in(Units.Degrees),
            null);

		addModuleToBuilder(builder, 0);
		addModuleToBuilder(builder, 1);
		addModuleToBuilder(builder, 2);
		addModuleToBuilder(builder, 3);
	}

    /** Telemeterizes a module */
	private void addModuleToBuilder(SendableBuilder builder, int module) {
        TelemetryManager.makeSendableTalonFX("Modules/" + module + "/Drive", 
            drivetrain.getModules()[module].getDriveMotor(), builder);
        
        TelemetryManager.makeSendableTalonFX("Modules/" + module + "/Angle", 
            drivetrain.getModules()[module].getSteerMotor(), builder);
	}
}