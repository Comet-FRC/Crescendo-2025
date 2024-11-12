package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.FeederSubsystem;

public class IndexNote extends Command {
    private final FeederSubsystem feeder;

    private double proximityDistanceMM = Double.MAX_VALUE;

    /**
     * Tries to adjust note position so that it's in the same spot every time. This
     * should probably be run in parallel with {@link PrepAmpCommand} so that as the
     * robot is revving its shooter motor, the feeder fixes the note
     */
    public IndexNote(FeederSubsystem feeder)
    {
        this.feeder = feeder;
    }

    @Override
    public void initialize() {
        feeder.setVelocity(100);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
}
