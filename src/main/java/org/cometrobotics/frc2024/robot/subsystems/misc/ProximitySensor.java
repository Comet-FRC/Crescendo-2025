package org.cometrobotics.frc2024.robot.subsystems.misc;

import org.cometrobotics.frc2024.robot.Constants;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

public class ProximitySensor {

    /* Singleton */
	
	private static ProximitySensor instance = null;
	
	public static ProximitySensor getInstance() {
		if (instance == null) instance = new ProximitySensor();
		return instance;
	}

	/* Implementation */

    private LaserCan laserCan;

    private ProximitySensor() {
        laserCan = new LaserCan(Constants.FEEDER.laserCanID);
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            
            System.out.println("LaserCAN configuration failed!");
        }
    }

    public double getDistanceMM() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();

		if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
			return 0;

		return measurement.distance_mm;
    }

    public boolean hasObject() {
        return getDistanceMM() <= 75;
    }

    public boolean isNoteIndexed() {
        return getDistanceMM() < 50;
    }
}
