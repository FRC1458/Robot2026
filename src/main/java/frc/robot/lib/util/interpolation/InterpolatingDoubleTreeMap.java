package frc.robot.lib.util.interpolation;

import java.util.Map.Entry;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class InterpolatingDoubleTreeMap extends InterpolatingTreeMap<Double, Double> {
    public InterpolatingDoubleTreeMap() {
        this(50);
    }

	public InterpolatingDoubleTreeMap(int max) {
        super(Interpolator.forDouble(), InverseInterpolator.forDouble(), Double::compare, max);
	}

    public InterpolatingDoubleTreeMap inverse() {
		var r = new InterpolatingDoubleTreeMap(max);
		for (Entry<Double, Double> entry : map.entrySet()) {
			r.put(entry.getValue(), entry.getKey());
		}

		return r;
    }
}
