package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.setVelocity(shooter.new ShooterSpeed(0, 0));
        Robot.getInstance().getRobotContainer().setRobotState(State.SHOOTING);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        Robot.getInstance().getRobotContainer().setRobotState(State.IDLE);
    }
}
