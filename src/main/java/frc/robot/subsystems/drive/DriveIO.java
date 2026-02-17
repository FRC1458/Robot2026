package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.io.CancoderIO;
import frc.robot.lib.io.TalonFXIO;
import frc.robot.subsystems.drive.ctre.CtreDrive;

public class DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public Pose2d fieldPose = new Pose2d();
        public ChassisSpeeds fieldSpeeds = new ChassisSpeeds();
        public String driveRequest = "";

        public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4]; 
        public SwerveModuleState[] moduleTargets = new SwerveModuleState[4];

        public String currentCommand = "";
        public String defaultCommand = "";
    }

    public class GyroIO {
        @AutoLog
        public static class GyroIOInputs {
            public boolean connected = false;
            public Rotation2d yawPosition = Rotation2d.kZero;
            public double yawVelocityRadPerSec = 0.0;
        }

        private final String name;
        private final GyroIOInputsAutoLogged inputs;
        private final StatusSignal<Angle> yaw ;
        private final StatusSignal<AngularVelocity> yawVelocity;

        public GyroIO(String name, Pigeon2 pigeon) {
            this.name = name;
            yaw = pigeon.getYaw();
            yawVelocity = pigeon.getAngularVelocityZWorld();
            inputs = new GyroIOInputsAutoLogged();
        }

        public void updateInputs() {
            inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
            inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
            inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
        }

        public void process() {
            Logger.processInputs(name, inputs);
        }
    }

    public static class ModuleIO {
        @AutoLog
        public static class ModuleIOInputs {
            public SwerveModuleState state;
            public SwerveModulePosition position; 
            public SwerveModuleState target;
        }

        private final String name;
        private final TalonFXIO driveMotorIO;
        private final TalonFXIO azimuthMotorIO;
        private final CancoderIO cancoderIO;
        private final ModuleIOInputsAutoLogged inputs;

        public ModuleIO(String name, SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.name = name;
            driveMotorIO = new TalonFXIO(name + "/DriveMotor", module.getDriveMotor());
            azimuthMotorIO = new TalonFXIO(name + "/AzimuthMotor", module.getSteerMotor());
            cancoderIO = new CancoderIO(name + "/Cancoder", module.getEncoder());
            inputs = new ModuleIOInputsAutoLogged();
        }

        /** Updates the set of loggable inputs. */
        public void updateInputs(SwerveModuleState state, SwerveModulePosition position, SwerveModuleState target) {
            inputs.state = state;
            inputs.position = position;
            inputs.target = target;

            driveMotorIO.updateInputs();
            azimuthMotorIO.updateInputs();
            cancoderIO.updateInputs();
        }

        public void process() {
            driveMotorIO.process();
            azimuthMotorIO.process();
            cancoderIO.process();
            Logger.processInputs(name, inputs);
        }
    }

    private final String name;
    private final ModuleIO[] moduleIOs;
    private final GyroIO gyroIO;
    private final CtreDrive drivetrain;
    private final DriveIOInputsAutoLogged inputs;

    public DriveIO(String name, CtreDrive drivetrain) {
        this.name = name;
        this.drivetrain = drivetrain;
        inputs = new DriveIOInputsAutoLogged();

        //FrontLeft, FrontRight, BackLeft, BackRight
        moduleIOs = new ModuleIO[] {
            new ModuleIO(name + "/Modules/" + "FL", drivetrain.getModule(0)),
            new ModuleIO(name + "/Modules/" + "FR", drivetrain.getModule(1)),
            new ModuleIO(name + "/Modules/" + "BL", drivetrain.getModule(2)),
            new ModuleIO(name + "/Modules/" + "BR", drivetrain.getModule(3))};

        gyroIO = new GyroIO(name + "/Gyro", drivetrain.getPigeon2());
    }

    public void updateInputs(SwerveRequest request, SwerveDriveState state, Command currentCommand, Command defaultCommand) {
        inputs.driveRequest = request.getClass().getSimpleName();
        inputs.fieldPose = state.Pose;
        inputs.fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            state.Speeds, state.Pose.getRotation());

        inputs.moduleStates = state.ModuleStates;
        inputs.modulePositions = state.ModulePositions;
        inputs.moduleTargets = state.ModuleTargets;

        inputs.currentCommand = currentCommand != null ? currentCommand.getName() : "None";
        inputs.defaultCommand = defaultCommand != null ? defaultCommand.getName() : "None";
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].updateInputs(state.ModuleStates[i], state.ModulePositions[i], state.ModuleTargets[i]);
        }

        gyroIO.updateInputs();
    }

    public void process() {
        for (int i = 0; i < 4; i++) {
            moduleIOs[i].process();
        }

        gyroIO.process();
        Logger.processInputs(name, inputs);
    }
}
