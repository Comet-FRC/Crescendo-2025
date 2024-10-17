package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private ShooterSubsystem shooter;
    private FeederSubsystem feeder;
    private Timer shootTimer = new Timer();

    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.feeder = feeder;
    }

    @Override
    public void initialize() {
        shootTimer.reset();
        shooter.shoot();
        feeder.intake();
        Robot.getInstance().getRobotContainer().setRobotState(State.SHOOTING);
    }

    @Override
    public void execute() {
        if (!Robot.getInstance().getRobotContainer().hasNote) {
            shootTimer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return shootTimer.hasElapsed(Constants.Shooter.postShotTimeout);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.getInstance().getRobotContainer().setRobotState(State.IDLE);
        shooter.stop();
        feeder.stop();
    }
}
