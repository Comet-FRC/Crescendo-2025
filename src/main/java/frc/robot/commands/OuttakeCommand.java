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

    public OuttakeCommand(ShooterSubsystem shooter, FeederSubsystem feeder, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;

        addRequirements(shooter, feeder, intake);
    }

    @Override
    public void initialize() {
        shooter.eject();
        feeder.eject();
        intake.eject();
        RobotContainer.setState(State.OUTTAKING);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stop();
        intake.stop();
        RobotContainer.setState(State.OUTTAKING);
    }
}
