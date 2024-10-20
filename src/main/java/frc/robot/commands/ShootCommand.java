package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Speed;

public class ShootCommand extends Command {
    ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.setVelocity(Speed.THROW);
        Robot.getInstance().getRobotContainer().setRobotState(State.SHOOTING);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        Robot.getInstance().getRobotContainer().setRobotState(State.IDLE);
    }
}
