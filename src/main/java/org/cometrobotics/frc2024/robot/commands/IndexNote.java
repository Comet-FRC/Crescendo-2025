package org.cometrobotics.frc2024.robot.commands;

import org.cometrobotics.frc2024.robot.subsystems.FeederSubsystem;
import org.cometrobotics.frc2024.robot.subsystems.misc.ProximitySensor;

import edu.wpi.first.wpilibj2.command.Command;

public class IndexNote extends Command {
    private final FeederSubsystem feeder;
    private final ProximitySensor laserCan;
    

    /**
     * Tries to adjust note position so that it's in the same spot every time. This
     * should probably be run in parallel with {@link PrepAmpCommand} so that as the
     * robot is revving its shooter motor, the feeder fixes the note
     */
    public IndexNote()
    {
        this.feeder = FeederSubsystem.getInstance();
        this.laserCan = ProximitySensor.getInstance();

        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setVelocity(100);
    }

    @Override
    public boolean isFinished() {
        return laserCan.isNoteIndexed();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
