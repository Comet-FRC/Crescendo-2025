package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;

    public IntakeCommand() {
        this.intake = IntakeSubsystem.getInstance();
        this.feeder = FeederSubsystem.getInstance();

        addRequirements(feeder, intake);
    }

    @Override
    public void initialize() {
        feeder.intake();
        intake.intake();
        RobotContainer.setState(State.INTAKING);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        intake.stop();
        RobotContainer.setState(State.IDLE);
    }
}
