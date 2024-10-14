package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class LimelightShooter extends Limelight {
    private double desiredDistance;

    public LimelightShooter(String name) {
        super(name);
    }

    public double proportionalX(double kP) {
        double drivingAdjustment = kP * getDistanceError();

		return drivingAdjustment;
    }

    /**
	 * simple proportional turning control with Limelight.
	 * "proportional control" is a control algorithm in which the output is proportional to the error.
	 * in this case, we are going to return an angular velocity that is proportional to the 
	 * "tx" value from the Limelight.*/
	public double aim_proportional(double kP) {
		// tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
		// your limelight 3 feed, tx should return roughly 31 degrees.
		double targetingAngularVelocity = tx() * kP;

		// convert to radians per second for our drive method
		targetingAngularVelocity *= Robot.getInstance().getRobotContainer().getSwerveSubsystem().getMaximumAngularVelocity();

		//invert since tx is positive when the target is to the right of the crosshair
		targetingAngularVelocity *= -1.0;

		return targetingAngularVelocity;
	}

    /**
	 * Returns the proportional value for turning to the speaker
     */
	public double getSpeakerAdjustment(Pose2d robotPose, double kP) {
		double x_displacement = robotPose.getX() - 1.442593;
		double y_displacement = 8.308975 - robotPose.getY();

		double targetAngle = 180 - Math.atan2(y_displacement,x_displacement);
		//SmartDashboard.putNumber("robot/target angle", targetAngle);
		double currentAngle = Robot.getInstance().getRobotContainer().getSwerveSubsystem().getSwerveDrive().getOdometryHeading().getDegrees();
		
		double turnAmount = targetAngle - currentAngle;
	
		double targetingAngularVelocity = turnAmount * kP;
		targetingAngularVelocity *= Robot.getInstance().getRobotContainer().getSwerveSubsystem().getMaximumAngularVelocity();
		targetingAngularVelocity *= -1.0;

		return targetingAngularVelocity;
	}

    /**
	 * @return distance from the target, in meters.
	 */
	public double getDistance() {
		Translation3d cameraTranslation = LimelightHelpers.getTargetPose3d_CameraSpace(getName()).getTranslation();
		
		double x = cameraTranslation.getX();
		double y = cameraTranslation.getY();
		double z = cameraTranslation.getZ();

		double distance = Math.sqrt(x*x + y*y + z*z);
		//System.out.println(distance);

		return distance;
	}

    @Override
    public double ty() {
        return LimelightHelpers.getTY(getName());
    }

    @Override
    public double tx() {
        return LimelightHelpers.getTX(getName());
    }

    public double getDistanceError() {
        return desiredDistance - getDistance();
    }

    public void updateDesiredDistance() {
        desiredDistance = SmartDashboard.getNumber("robot/desired distance", 2.3);
    }

    public double getDesiredDistance() {
        return desiredDistance;
    }
}
