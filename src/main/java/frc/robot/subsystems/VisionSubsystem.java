package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

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

    /**
	 * simple proportional turning control with Limelight.
	 * "proportional control" is a control algorithm in which the output is proportional to the error.
	 * in this case, we are going to return an angular velocity that is proportional to the 
	 * "tx" value from the Limelight.*/
	public double aim_proportional()
	{    
		double kP = .035;

		// tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
		// your limelight 3 feed, tx should return roughly 31 degrees.
		double targetingAngularVelocity = LimelightHelpers.getTX(name) * kP;

		// convert to radians per second for our drive method
		targetingAngularVelocity *= Robot.getInstance().getRobotContainer().getSwerveSubsystem().getMaximumAngularVelocity();

		//invert since tx is positive when the target is to the right of the crosshair
		targetingAngularVelocity *= -1.0;

		return targetingAngularVelocity;
	}


/**
	 * simple proportional ranging control with Limelight's "ty" value
	 */
	/*double limelight_range_proportional()
	{    
		double kP = 0.1;

		double targetingForwardSpeed = LimelightHelpers.getTY("limelight-shooter") * kP;
		targetingForwardSpeed *= swerve.getMaximumVelocity();
		targetingForwardSpeed *= -1.0;

		// TODO: this is bad but we will fix later
		targetingForwardSpeed -= 4.3;
		//double out = targetingForwardSpeed > 0 ? targetingForwardSpeed : 0;
		return targetingForwardSpeed;
	}*/

	/**
	 * simple proportional ranging control with Limelight's "ty" value
     * @param targetHeight the distance, in inches, from the floor to target
	 */
	public double range_proportional(double targetHeight)
	{    
		double kP = 0.1;

		// TODO: Measure apriltag height
		double currentDistance = getDistanceFromTarget(targetHeight);
		double desiredDistance = 2; // Meters
		double distanceError = desiredDistance - currentDistance;
		double drivingAdjustment = kP * distanceError;

		return drivingAdjustment;
	}
}
