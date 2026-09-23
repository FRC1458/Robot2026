package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.lib.field.FieldLayout;
import frc.robot.lib.subsystem.LoggedSubsystem;
import frc.robot.lib.util.TunableNumber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants.VisionDeviceConstants;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;

public class VisionDeviceManager extends LoggedSubsystem {
	private static final TunableNumber timestampOffset =
			new TunableNumber("VisionTimestampOffset", 0.1, false);

	private final VisionDevice frontRightCamera;
	private final VisionDevice frontLeftCamera;
	private final List<VisionDevice> cameras;

	private VisionSystemSim visionSim;
	public Drive drive;

	public VisionDeviceManager(Drive drive) {
		super();
		this.drive = drive;
		frontRightCamera =
				new VisionDevice(VisionDeviceConstants.FR_CONSTANTS, drive, getName() + "/FrontRight");
		frontLeftCamera =
				new VisionDevice(VisionDeviceConstants.FL_CONSTANTS, drive, getName() + "/FrontLeft");

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
	}

	@Override
	public void simulationPeriodic() {
		if (visionSim != null) {
			visionSim.update(drive.getPose());
		}
	}

	public synchronized boolean isFullyConnected() {
		return cameras.stream().allMatch(VisionDevice::isConnected);
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

	public double getTimestampOffset() {
		return timestampOffset.get();
	}

	@Override
	public void log() {
		cameras.forEach((c) -> c.log());
	}
}
