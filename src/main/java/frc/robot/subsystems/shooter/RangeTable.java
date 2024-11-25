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
		RANGE_TABLE_SPEAKER.put(0.91186, new ShooterSpeed(1200, 3200));
		RANGE_TABLE_SPEAKER.put(1.19126, new ShooterSpeed(1500, 2500));
		RANGE_TABLE_SPEAKER.put(1.5113, new ShooterSpeed(2200, 2200));
		RANGE_TABLE_SPEAKER.put(1.83388, new ShooterSpeed(2700, 2200));
		RANGE_TABLE_SPEAKER.put(2.12089, new ShooterSpeed(3000, 1900));
		RANGE_TABLE_SPEAKER.put(2.43178, new ShooterSpeed(2900, 1700));
		RANGE_TABLE_SPEAKER.put(2.74574, new ShooterSpeed(2800, 1550));
		RANGE_TABLE_SPEAKER.put(3.07186, new ShooterSpeed(2800, 1425));
		RANGE_TABLE_SPEAKER.put(3.35328, new ShooterSpeed(2700, 1400));

		// TODO: MEASURE RANGE TABLE VALUES
		// THIS IS THE ONLY ONE WE KNOW FOR SURE WORKS RN
        RANGE_TABLE_SPEAKER.put(2.34, new ShooterSpeed(3100, 3100));
	}

    public ShooterSpeed get(double distance) {
        return this.RANGE_TABLE_SPEAKER.get(distance);
    }
}
