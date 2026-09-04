package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.lib.field.FieldLayout;
import frc.robot.lib.util.MovingAverageDouble;
import frc.robot.lib.util.TunableNumber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants.VisionDeviceConstants;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;

public class VisionDeviceManager extends SubsystemBase {
	private static boolean visionDisabled = false;

	private static final TunableNumber timestampOffset =
			new TunableNumber("VisionTimestampOffset", 0.1, false);

	private final VisionDevice frontRightCamera;
	private final VisionDevice frontLeftCamera;
	private final List<VisionDevice> cameras;

	private final MovingAverageDouble headingAvg = new MovingAverageDouble(100);
	private double movingAvgRead = 0.0;

	private VisionSystemSim visionSim;
	public Drive drive;

	private VisionDeviceManager(Drive drive) {
		this.drive = drive;
		frontRightCamera = new VisionDevice(VisionDeviceConstants.FR_CONSTANTS, drive);
		frontLeftCamera = new VisionDevice(VisionDeviceConstants.FL_CONSTANTS, drive);

		cameras = List.of(frontRightCamera, frontLeftCamera);

		if (Robot.isSimulation()) {
			visionSim = new VisionSystemSim(getName());
			visionSim.addAprilTags(FieldLayout.APRILTAG_MAP);
			cameras.forEach(
					camera ->
							visionSim.addCamera(camera.getSimulation(), camera.getConstants().robotToCamera));
		}
	}

	@Override
	public void periodic() {
		cameras.forEach(VisionDevice::periodic);
		movingAvgRead = headingAvg.getAverage();
	}

	@Override
	public void simulationPeriodic() {
		if (visionSim != null) {
			visionSim.update(drive.getPose());
		}
	}

	public double getMovingAvgRead() {
		return movingAvgRead;
	}

	public synchronized MovingAverageDouble getMovingAverage() {
		return headingAvg;
	}

	public synchronized boolean isFullyConnected() {
		// TODO: Replace with actual connection check if needed
		// return cameras.stream().allMatch(VisionDevice::isConnected);
		return true;
	}

	public Command bootUp() {
		Command[] bootCommands =
				cameras.stream().map(VisionDevice::bootUpSequence).toArray(Command[]::new);

		return Commands.parallel(bootCommands)
				.withTimeout(4.0)
				.andThen(Commands.print("Finished vision bootup"));
	}

	public synchronized VisionDevice getFrontRightVision() {
		return frontRightCamera;
	}

	public synchronized VisionDevice getFrontLeftVision() {
		return frontLeftCamera;
	}

	public static double getTimestampOffset() {
		return timestampOffset.get();
	}

	public static boolean isVisionDisabled() {
		return visionDisabled;
	}

	public static void setVisionDisabled(boolean disabled) {
		visionDisabled = disabled;
	}
}
