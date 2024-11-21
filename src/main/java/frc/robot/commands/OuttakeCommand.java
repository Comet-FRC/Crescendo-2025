package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OuttakeCommand extends Command {
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final IntakeSubsystem intake;

    public OuttakeCommand() {
        this.shooter = ShooterSubsystem.getInstance();
        this.feeder = FeederSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.addRequirements(shooter, feeder, intake);
    }

    @Override
    public void initialize() {
        this.shooter.eject();
        this.feeder.eject();
        this.intake.eject();
        RobotContainer.setState(State.OUTTAKING);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
        this.feeder.stop();
        this.intake.stop();
        RobotContainer.setState(State.OUTTAKING);
    }
}
