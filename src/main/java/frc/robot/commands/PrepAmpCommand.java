package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.LimelightShooter;

public class PrepAmpCommand extends Command {

    private final ShooterSubsystem shooter;
    private final LimelightShooter limelightShooter;

    private Pose3d targetPose;

    public PrepAmpCommand(ShooterSubsystem shooter, LimelightShooter limelightShooter) {
        this.shooter = shooter;
        this.limelightShooter = limelightShooter;
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        boolean isRedAlliance = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;

        int priorityTagID;

        if (isRedAlliance) priorityTagID = 5;
        else priorityTagID = 6;

        LimelightHelpers.setPriorityTagID(getName(), priorityTagID);
        
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        targetPose = aprilTagFieldLayout.getTagPose(priorityTagID).get();

        shooter.amp();
        Robot.getInstance().getRobotContainer().setRobotState(State.PREPPING);
    }

    @Override
    public void execute() {
    }


    @Override
    public boolean isFinished() {
        // if there is no target, then obviously shoot prep isn't done
        if (!limelightShooter.hasTarget()) {
            return false;
        }
        return isReadyToShoot();
    }

    private boolean isReadyToShoot() {
        double distanceError = limelightShooter.getDistanceError();
		ChassisSpeeds robotVelocity = Robot.getInstance().getRobotContainer().getSwerveSubsystem().getRobotVelocity();

		// TODO: Check if these values look good
		return
			Math.abs(distanceError) < 0.025 &&
			Math.abs(robotVelocity.vxMetersPerSecond) < 0.01 &&
			Math.abs(robotVelocity.vyMetersPerSecond) < 0.01 &&
			Math.abs(robotVelocity.omegaRadiansPerSecond) < 4 &&
			shooter.isReady(false);
	}

    @Override
    public void end(boolean interrupted) {
        // if the user stops pressing button
        if (interrupted) {
            shooter.stop();
        }
        Robot.getInstance().getRobotContainer().setRobotState(State.IDLE);
    }
}

