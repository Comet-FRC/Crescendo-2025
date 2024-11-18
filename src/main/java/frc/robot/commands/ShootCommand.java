package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(shooter.new ShooterSpeed(0, 0));
        RobotContainer.setState(State.SHOOTING);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        RobotContainer.setState(State.IDLE);
    }
}
