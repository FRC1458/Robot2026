package frc.robot.lib.field;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.util.interpolation.InterpolatingDoubleTreeMap;

// 5th-ish try
public class Sotm {
	InterpolatingDoubleTreeMap tof;
	InterpolatingDoubleTreeMap inv;

	public Sotm(InterpolatingDoubleTreeMap tof) {
		this.tof = tof;
		inv = tof.inverse();
	}

	public Translation2d solve(Translation2d pos, Translation2d tar, Translation2d spd) {
		Translation2d virtual = tar;
		for (int i = 0; i < 5; i++) {
			Translation2d delta = virtual.minus(pos);
			double time = tof.get(delta.getNorm());
			virtual = tar.minus(spd.times(time));
		}
		return virtual;
	}
}
