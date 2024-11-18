package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    IntakeSubsystem intake;
    FeederSubsystem feeder;

    public IntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder) {
        this.intake = intake;
        this.feeder = feeder;
    }

    @Override
    public void initialize() {
        feeder.intake();
        intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        intake.stop();
    }
}
