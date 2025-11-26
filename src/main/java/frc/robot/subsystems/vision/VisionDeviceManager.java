package frc.robot.subsystems.vision;

import frc.robot.Constants;
import frc.robot.lib.util.TunableNumber;
import frc.robot.lib.util.MovingAverage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

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

	private MovingAverage<Double> headingAvg = new MovingAverage<Double>(
		100, Double.valueOf(0), 
		(Double x, Double y) -> { return x + y; }, (Double x, Integer y) -> { return x / y; });
	private double movingAvgRead = 0.0;

	private static boolean disable_vision = false;

	public VisionDeviceManager() {
		// leftCamera = new VisionDevice(Constants.Limelight.VisionDeviceConstants.L_CONSTANTS);
		// rightCamera = new VisionDevice(Constants.Limelight.VisionDeviceConstants.R_CONSTANTS);
		frontrCamera = new VisionDevice(Constants.Limelight.VisionDeviceConstants.FR_CONSTANTS);
		frontlCamera = new VisionDevice(Constants.Limelight.VisionDeviceConstants.FL_CONSTANTS);
		cameras = List.of(frontrCamera, frontlCamera);
		// cameras = List.of(rightCamera);
	}

	@Override
	public void periodic() {
		cameras.forEach(VisionDevice::periodic);
		movingAvgRead = headingAvg.getAverage();
		SmartDashboard.putNumber("Vision heading moving avg", getMovingAvgRead());
		SmartDashboard.putBoolean("vision disabled", visionDisabled());
	}

	public Double getMovingAvgRead() {
		return movingAvgRead;
	}

	public synchronized MovingAverage<Double> getMovingAverage() {
		return headingAvg;
	}

	public synchronized boolean isFullyConnected() {
		return frontlCamera.isConnected()
			&& frontrCamera.isConnected();
			// && rightCamera.isConnected();
			// && backCamera.isConnected();
	}

	public synchronized boolean inRange() {
		return frontlCamera.inSnapRange() && frontrCamera.hasTarget();
	}

	// public synchronized VisionDevice getLeftVision() {
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

	public static boolean visionDisabled() {
		return disable_vision;
	}

	public static void setDisableVision(boolean disable) {
		disable_vision = disable;
	}
}
