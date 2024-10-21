package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.FeederSubsystem;

public class IndexNote extends Command {
    private final FeederSubsystem feeder;
    private final LaserCan laserCan;

    private RobotContainer robotContainer;

    private double proximityDistanceMM = Double.MAX_VALUE;

    /**
     * Tries to adjust note position so that it's in the same spot every time. This
     * should probably be run in parallel with {@link PrepAmpCommand} so that as the
     * robot is revving its shooter motor, the feeder fixes the note
     */
    public IndexNote(FeederSubsystem feeder, LaserCan laserCan)
    {
        this.feeder = feeder;
        this.laserCan = laserCan;
    }

    @Override
    public void initialize() {
        robotContainer = Robot.getInstance().getRobotContainer();
    }

    @Override
    public void execute() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();
		if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
			return;

		proximityDistanceMM = measurement.distance_mm;

        //robotContainer.setNoteStatus(true);
        if (proximityDistanceMM < 51) {
            feeder.setVelocity(100);
        } else {
            feeder.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return proximityDistanceMM >= 51;
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        if (interrupted)
            robotContainer.setRobotState(State.IDLE);
    }
}
