package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision.LimelightShooter;

public class PrepShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final LimelightShooter limelightShooter;

    public PrepShootCommand(ShooterSubsystem shooter, LimelightShooter limelightShooter) {
        this.shooter = shooter;
        this.limelightShooter = limelightShooter;
    }

    @Override
    public void initialize() {
        shooter.shoot();
        Robot.getInstance().getRobotContainer().setRobotState(State.PREPPING);
    }

    @Override
    public void execute() {
        if (!limelightShooter.hasTarget()) return;

        double rotationalSpeed = limelightShooter.aim_proportional(Constants.SPEAKER_AIM_KP);
        double forwardSpeed = limelightShooter.forward_proportional(Constants.SPEAKER_APPROACH_KP);

        Robot.getInstance().getRobotContainer().setForwardSpeedOverride(forwardSpeed);
        Robot.getInstance().getRobotContainer().overrideRotationalSpeed(rotationalSpeed);
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

