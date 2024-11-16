package frc.robot.subsystems;

import java.util.logging.Level;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class LaserCanSubsystem extends SubsystemBase {
    private LaserCan laserCan;

    public LaserCanSubsystem() {
        laserCan = new LaserCan(Constants.Feeder.laserCanID);
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            Robot.getLogger().log(Level.SEVERE, "LaserCAN configuration failed!");
        }
    }

    public double getDistanceMM() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();

		if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
			return 0;

        double distanceMeters = measurement.distance_mm;
		return distanceMeters;
    }

    public boolean hasObject() {
        return getDistanceMM() <= 75;
    }

    public boolean isNoteIndexed() {
        return getDistanceMM() < 50;
    }
}
