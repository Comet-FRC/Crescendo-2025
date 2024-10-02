package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;

public class VisionSubsystem {
    private final String name;

    /**
     * the angle, in degrees, at which the limelight is mounted
     * (the angle back from the vertical)
     */
    private final double mountAngle;

    /**
     * The distance, in inches, from the center of the lens to the floor
     */
    private final double height;
    
    public VisionSubsystem(String name, double mountAngle, double height) {
        this.name = name;
        this.mountAngle = mountAngle;
        this.height = height;
    }

    /**
     * Returns the horizontal distance, in inches, of the limelight to the target
     * @param ty the vertical offset angle of the target
     * @param targetHeight the distance, in inches, of the target to the floor
     */
    public double getDistanceFromTarget(double targetHeight) {
        double angleToGoalDegrees = mountAngle + LimelightHelpers.getTY(name);
        double angleToGoalRadians = Units.degreesToRadians(angleToGoalDegrees);

        double distanceFromTarget = (targetHeight - height) / Math.tan(angleToGoalRadians);
        return distanceFromTarget;
    }
}
