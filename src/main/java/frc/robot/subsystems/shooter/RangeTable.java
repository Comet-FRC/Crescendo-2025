package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class RangeTable {

    private final InterpolatingTreeMap<Double, ShooterSpeed> RANGE_TABLE_SPEAKER;
     
    public RangeTable() {
        this.RANGE_TABLE_SPEAKER = new InterpolatingTreeMap<Double, ShooterSpeed>(
            InverseInterpolator.forDouble(),
			ShooterSpeed.getInterpolator()
        );

        this.setupRangeTable();
    }

    private void setupRangeTable() {
		RANGE_TABLE_SPEAKER.put(1.144, new ShooterSpeed(6000, 4500));
		RANGE_TABLE_SPEAKER.put(1.33, new ShooterSpeed(3600, 3400));
		RANGE_TABLE_SPEAKER.put(1.48, new ShooterSpeed(3500, 3500));
		RANGE_TABLE_SPEAKER.put(1.87, new ShooterSpeed(3000, 2800));
		RANGE_TABLE_SPEAKER.put(2.1203355593490785, new ShooterSpeed(2800, 2800));
		RANGE_TABLE_SPEAKER.put(2.3587506681318846, new ShooterSpeed(2700, 2600));
		RANGE_TABLE_SPEAKER.put(2.534346332294663, new ShooterSpeed(2700, 2350));
	}

    public ShooterSpeed get(double distance) {
        return this.RANGE_TABLE_SPEAKER.get(distance);
    }
}
