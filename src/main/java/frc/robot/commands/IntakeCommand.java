package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision.LimelightIntake;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final LimelightIntake limelight;
    private final LaserCan laserCan;

    private final RobotContainer robotContainer;

    /**
     * The difference between the desired value of the LaserCAN to the measured value
     */
    private double deltaDistance = 0;

    public IntakeCommand(
        IntakeSubsystem intake,
        FeederSubsystem feeder,
        LimelightIntake limelight,
        LaserCan laserCan) {
        this.intake = intake;
        this.feeder = feeder;
        this.limelight = limelight;
        this.laserCan = laserCan;

        robotContainer = Robot.getInstance().getRobotContainer();
    }

    @Override
    public void initialize() {
        robotContainer.setRobotState(State.INTAKING);
    }

    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            robotContainer.setRotationalSpeedOverride(0);
            double strafeSpeed = MathUtil.applyDeadband(limelight.strafe_proportional(Constants.INTAKE_STRAFE_KP), 0.06);
            robotContainer.setStrafeSpeedOverride(strafeSpeed);

            if (robotContainer.getSwerveSubsystem().getSwerveDrive().getRobotVelocity().vyMetersPerSecond < 0.01) {
                double forwardSpeed = -2;
                robotContainer.setForwardSpeedOverride(forwardSpeed);
            } 
        }

        LaserCan.Measurement measurement = laserCan.getMeasurement();
		if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
			return;

		boolean hasNote = measurement.distance_mm <= 75;

        if (!hasNote) {
            intake.intake();
            feeder.intake();
        } else {
            //TODO: set desired distance
            double desiredDistance = 50;
            double kP = 0.01;

            deltaDistance = desiredDistance - measurement.distance_mm;
            double correction = kP * deltaDistance;
            correction *= -1;
            feeder.setFeederSpeed(correction);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: some threshold value here
        return Math.abs(deltaDistance) < 10;
    }

    @Override
    public void end(boolean interrupted) {
        robotContainer.setNoteStatus(true);
        intake.stop();
        feeder.stop();
        robotContainer.setRobotState(State.IDLE);
    }
}
