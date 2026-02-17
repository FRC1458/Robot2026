package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

public class VisionIO {
	@AutoLog
	public static class VisionIOInputs {
		public boolean enabled = true;
		public boolean fullyConnected = false;

		public double timestampOffset = 0.0;
		public double headingMovingAverage = 0.0;

		public String currentCommand = "";
		public String defaultCommand = "";
	}

	public static class VisionDeviceIO {
		@AutoLog
		public static class VisionDeviceIOInputs {
			public boolean connected = false;
			public boolean hasTarget = false;
			public Pose2d estimatedPose = Pose2d.kZero;
			public double lastTimestamp = 0.0;
		}

		private final String name;
		private final VisionDevice device;
		private final VisionDeviceIOInputsAutoLogged inputs;

		public VisionDeviceIO(String name, VisionDevice device) {
			this.name = name;
			this.device = device;
			this.inputs = new VisionDeviceIOInputsAutoLogged();
		}

		public void updateInputs() {
			inputs.connected = device.isConnected();
			inputs.hasTarget = device.hasTarget();
			inputs.estimatedPose = device.botPose != null ? device.botPose : Pose2d.kZero;
		}

		public void process() {
			Logger.processInputs(name, inputs);
		}
	}

	private final String name;
	private final VisionDeviceManager manager;
	private final VisionDeviceIO[] cameraIOs;
	private final VisionIOInputsAutoLogged inputs;

	public VisionIO(String name, VisionDeviceManager manager) {
		this.name = name;
		this.manager = manager;
		this.inputs = new VisionIOInputsAutoLogged();

		List<VisionDevice> devices = manager.cameras;

		cameraIOs = new VisionDeviceIO[devices.size()];
		for (int i = 0; i < devices.size(); i++) {
			cameraIOs[i] = new VisionDeviceIO(
				name + "/Cameras/" + devices.get(i).getConstants().tableName,
				devices.get(i));
		}
	}

	public void updateInputs(Command currentCommand, Command defaultCommand) {
		inputs.enabled = !VisionDeviceManager.getVisionDisabled();
		inputs.timestampOffset = VisionDeviceManager.getTimestampOffset();
		inputs.fullyConnected = manager.isFullyConnected();
		inputs.headingMovingAverage = manager.getMovingAvgRead();

		inputs.currentCommand = currentCommand != null ? currentCommand.getName() : "None";
		inputs.defaultCommand = defaultCommand != null ? defaultCommand.getName() : "None";

		for (VisionDeviceIO cameraIO : cameraIOs) {
			cameraIO.updateInputs();
		}
	}

	public void process() {
		for (VisionDeviceIO cameraIO : cameraIOs) {
			cameraIO.process();
		}

		Logger.processInputs(name, inputs);
	}
}
