package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final String limelightName;

    // Constructor accepting the swerve subsystem's pose estimator
    public VisionSubsystem(SwerveSubsystem swerve, String limelightName) {
        this.poseEstimator = swerve.getPoseEstimator();  // Store pose estimator reference
        this.limelightName = limelightName;              // Store limelight name
    }

    // Update pose based on LimeLight vision measurement
    public void updateVisionPose() {
        // Get pose estimate from LimeLight
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        
        // Ensure the data is valid and we have detected at least 2 tags
        if (limelightMeasurement != null && limelightMeasurement.tagCount >= 2) {
            // Set vision measurement standard deviations (trust more for position, less for rotation)
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            
            // Update the pose estimator with the new vision measurement
            poseEstimator.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds
            );

            Robot.getLogger().info("Updated Vision Pose!");
        }
    }

    public Pose2d getTagPose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        
        if (limelightMeasurement != null && limelightMeasurement.tagCount >= 1) {
            return limelightMeasurement.pose;
        }
        return null; // or return a default Pose2d if no tag is detected
    }

    @Override
    public void periodic() {
        // Periodically update the vision-based pose estimate
        updateVisionPose();
    }
}
