package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import frc.robot.lib.util.TunableNumber;
import frc.robot.subsystems.TelemetryManager;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants.VisionDeviceConstants;
import frc.robot.Robot;
import frc.robot.lib.field.FieldLayout;
import frc.robot.lib.util.MovingAverageDouble;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.simulation.VisionSystemSim;

public class VisionDeviceManager extends SubsystemBase {
    public static boolean enabled;
	public static VisionDeviceManager visionDeviceManagerInstance;
	public static VisionDeviceManager getInstance() {
		if (visionDeviceManagerInstance == null) {
			visionDeviceManagerInstance = new VisionDeviceManager();
		}
		return visionDeviceManagerInstance;
	}

	// private VisionDevice leftCamera;
	@SuppressWarnings("unused")
	private VisionDevice rightCamera;
	private VisionDevice frontrCamera;
	private VisionDevice frontlCamera;

	private List<VisionDevice> cameras;

	private static TunableNumber timestampOffset = new TunableNumber("VisionTimestampOffset", (0.1), false);

	private MovingAverageDouble headingAvg = new MovingAverageDouble(100);
	private double movingAvgRead = 0.0;

	private static boolean visionDisabled = false;

	public VisionSystemSim visionSim;

	public VisionDeviceManager() {
		// leftCamera = new VisionDevice(Constants.Limelight.VisionDeviceConstants.L_CONSTANTS);
		// rightCamera = new VisionDevice(Constants.Limelight.VisionDeviceConstants.R_CONSTANTS);
		frontrCamera = new VisionDevice(VisionDeviceConstants.FR_CONSTANTS);
		frontlCamera = new VisionDevice(VisionDeviceConstants.FL_CONSTANTS);
		cameras = List.of(frontrCamera, frontlCamera);
		// cameras = List.of(rightCamera);
		if (Robot.isSimulation()) {
			visionSim = new VisionSystemSim(getName());
			visionSim.addAprilTags(FieldLayout.APRILTAG_MAP);
			cameras.forEach((camera) -> visionSim.addCamera(camera.getSimulation(), camera.getConstants().robotToCamera));
		}
		TelemetryManager.getInstance().addSendable(this);
	}

	@Override
	public void periodic() {
		if (Robot.isSimulation()) {
			visionSim.update(Drive.getInstance().getPose());
		}
		cameras.forEach(VisionDevice::periodic);
		movingAvgRead = headingAvg.getAverage();
		SmartDashboard.putNumber("Vision heading moving avg", getMovingAvgRead());
		SmartDashboard.putBoolean("vision disabled", getVisionDisabled());
	}

	public double getMovingAvgRead() {
		return movingAvgRead;
	}

	public synchronized MovingAverageDouble getMovingAverage() {
		return headingAvg;
	}

	public synchronized boolean isFullyConnected() {
		return frontlCamera.isConnected()
			&& frontrCamera.isConnected();
			// && rightCamera.isConnected();
			// && backCamera.isConnected();
	}

	public Command bootUp() {
		return Commands.parallel(
			frontlCamera.bootUpSequence(),
			frontrCamera.bootUpSequence())
			.withTimeout(4)
			.andThen(Commands.print("Finished vision bootup"));
	}

	// public synchronized VisionDevice getLeftVision() {z
	// 	return leftCamera;
	// }

	// public synchronized VisionDevice getRightVision() {
	// 	return rightCamera;
	// }

	public synchronized VisionDevice getFrontRVision() {
		return frontrCamera;
	}

	public synchronized VisionDevice getFrontLVision() {
		return frontlCamera;
	}

	public static double getTimestampOffset() {
		return timestampOffset.get();
	}

	public static boolean getVisionDisabled() {
		return visionDisabled;
	}

	public static void setDisableVision(boolean disable) {
		visionDisabled = disable;
	}
}
