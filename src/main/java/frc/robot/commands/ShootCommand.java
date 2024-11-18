package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSpeed;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        double topSpeed = SmartDashboard.getNumber("robot/shooter/topSpeed", 0);
        double bottomSpeed = SmartDashboard.getNumber("robot/shooter/bottomSpeed", 0);
        ShooterSpeed speed = new ShooterSpeed(topSpeed, bottomSpeed);
        shooter.setVelocity(speed);
        RobotContainer.setState(State.SHOOTING);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        RobotContainer.setState(State.IDLE);
    }
}
