package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.misc.ProximitySensor;

/*
 * Shoots the note. Assumes the shooter is revved.
 */
public class ShootNoteRevvedShooter extends Command {
    private final FeederSubsystem feeder;
    private final ShooterSubsystem shooter;

    public ShootNoteRevvedShooter() {
        this.feeder = FeederSubsystem.getInstance();
        this.shooter = ShooterSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        feeder.intake();
    }

    @Override
    public boolean isFinished() {
        return !ProximitySensor.getInstance().isNoteIndexed();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        shooter.stop();
    }
}
