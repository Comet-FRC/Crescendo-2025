package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision.LimelightIntake;

public class AutoStopIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final LaserCan laserCan;

    public AutoStopIntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder, LaserCan laserCan) {
        this.intake = intake;
        this.feeder = feeder;
        this.laserCan = laserCan;
    }

    @Override
    public void initialize() {
        feeder.intake();
        intake.intake();
        Robot.getInstance().getRobotContainer().setRobotState(State.INTAKING);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        intake.stop();
        Robot.getInstance().getRobotContainer().setRobotState(State.IDLE);
    }
}
