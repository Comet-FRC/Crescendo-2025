package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName;
    private final SwerveSubsystem swerve;

    /**
     * 1 if valid target exists. 0 if no valid targets exist
     */
    private int tv;

    /**
     * Horiz. Offset From Crosshair To Target
     * (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    private double tx;
    
    /**
     * Vertical Offset From Crosshair To Target
     * (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    private double ty; 



    // Constructor accepting the swerve subsystem's pose estimator
    public VisionSubsystem(SwerveSubsystem swerve, String limelightName) {
        this.limelightName = limelightName; // Store limelight name
        this.swerve = swerve;
    }

    public void updateVision() {

    }
}
