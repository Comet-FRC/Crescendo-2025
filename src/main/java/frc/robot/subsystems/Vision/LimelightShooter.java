package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.List;
import java.util.logging.Level;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class LimelightShooter extends Limelight {
    private double desiredDistance;

    public LimelightShooter(String name) {
        super(name);
    }

    public double forward_proportional(double kP) {
        double drivingAdjustment = kP * getDistanceError();
		drivingAdjustment *= -1;
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
		double targetingAngularVelocity = getTX() * kP;
		targetingAngularVelocity *= Robot.getInstance().getRobotContainer().getSwerveSubsystem().getMaximumAngularVelocity();
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
    protected double getRawTX() {
        return LimelightHelpers.getTX(getName());
    }

    @Override
    protected double getRawTY() {
        return LimelightHelpers.getTY(getName());
    }

    public double getDistanceError() {
        return getDesiredDistance() - getDistance();
    }

    public void updateDesiredDistance() {
		double newDistance = SmartDashboard.getNumber("robot/desired speaker distance", Constants.SPEAKER_DISTANCE);

		if (desiredDistance == newDistance) {
			return;
		}
        desiredDistance = newDistance;
		Robot.getLogger().log(Level.FINEST, "desired speaker distance set to " + newDistance);
    }
	
	public double getDesiredDistance() {
		Pose3d pose = LimelightHelpers.getCameraPose3d_TargetSpace(getName());
		double x = pose.getX();
		double y = pose.getY();
		double deltaAngle = Math.abs(Units.radiansToDegrees(Math.atan2(x, y)));
		SmartDashboard.putNumber("robot/shooter delta angle", deltaAngle);
		return desiredDistance - 0.1 * (1-deltaAngle/50.0);
	}
}
