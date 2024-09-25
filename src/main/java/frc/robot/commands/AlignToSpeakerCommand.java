package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignToSpeakerCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private Pose2d tagPose;

    private boolean isTag = false;
    boolean isAligned;

    public AlignToSpeakerCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        isAligned = false;

        tagPose = visionSubsystem.getTagPose();
        
        if (tagPose != null) {
            // Calculate the desired pose for the robot to aim at the speaker
            Pose2d targetPose = calculateTargetPose(tagPose);

            // Drive the robot to the calculated target pose
            swerveSubsystem.driveToPose(targetPose);
        }
    }

    @Override
    public void execute() {
        tagPose = visionSubsystem.getTagPose();
    }

    private Pose2d calculateTargetPose(Pose2d tagPose) {
        // Adjust the target pose based on the desired offset from the tag
        // Example: 1 meter forward and aligned with the tag
        double xOffset = 1.0; // Change this value as needed
        double yOffset = 0.0; // Change this value as needed
        Rotation2d targetRotation = Rotation2d.fromDegrees(0); // Change this to your desired angle

        return new Pose2d(
            tagPose.getX() + xOffset * Math.cos(tagPose.getRotation().getRadians()),
            tagPose.getY() + yOffset * Math.sin(tagPose.getRotation().getRadians()),
            targetRotation
        );
    }

    public double calculateDistance(Pose2d pose1, Pose2d pose2) {
        double dx = pose2.getX() - pose1.getX();
        double dy = pose2.getY() - pose1.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = swerveSubsystem.getPose();
    
        return calculateDistance(currentPose, tagPose) < 3;
    }
}
