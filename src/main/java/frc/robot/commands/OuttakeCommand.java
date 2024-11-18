package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    }

    @Override
    public void initialize() {
        shooter.eject();
        feeder.eject();
        intake.eject();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        feeder.stop();
        intake.stop();
    }
}
