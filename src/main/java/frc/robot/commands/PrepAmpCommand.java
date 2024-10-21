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
import frc.robot.subsystems.Vision.LimelightAmp;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.LimelightShooter;

public class PrepAmpCommand extends Command {

    private final ShooterSubsystem shooter;
    private final LimelightAmp limelightAmp;

    public PrepAmpCommand(ShooterSubsystem shooter, LimelightAmp limelightAmp) {
        this.shooter = shooter;
        this.limelightAmp = limelightAmp;
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance();
        boolean isRedAlliance = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;

        if (isRedAlliance) LimelightHelpers.setPriorityTagID(limelightAmp.getName(), 5);
        else LimelightHelpers.setPriorityTagID(limelightAmp.getName(), 6);

        shooter.amp();
        Robot.getInstance().getRobotContainer().setRobotState(State.PREPPING);
    }

    @Override
    public void execute() {
        if (!limelightAmp.hasTarget()) return;

        double rotationalSpeed = limelightAmp.aim_proportional(Constants.SPEAKER_AIM_KP);
        double forwardSpeed = limelightAmp.forward_proportional(Constants.AMP_APPROACH_KP);

        Robot.getInstance().getRobotContainer().setForwardSpeedOverride(forwardSpeed);
        Robot.getInstance().getRobotContainer().overrideRotationalSpeed(rotationalSpeed);
    }


    @Override
    public boolean isFinished() {
        return isReadyToShoot();
    }

    private boolean isReadyToShoot() {
        double distanceError = limelightAmp.getDistanceError();
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

