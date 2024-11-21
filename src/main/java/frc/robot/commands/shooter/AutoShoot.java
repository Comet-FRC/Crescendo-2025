package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.IndexNote;
import frc.robot.commands.drivetrain.AimCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShoot extends SequentialCommandGroup {

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;

    private Pose2d targetPose;

    public AutoShoot() {
        this.swerve = SwerveSubsystem.getInstance();
        this.shooter = ShooterSubsystem.getInstance();
        this.feeder = FeederSubsystem.getInstance();
        this.addRequirements(swerve, shooter, feeder);

        targetPose = new Pose2d(swerve.getSpeakerPosition(), new Rotation2d());

        this.addCommands(
            new ParallelCommandGroup(
                new AimCommand(() -> this.swerve.getSpeakerPosition()),
                new SequentialCommandGroup(
                    new IndexNote(),
                    this.shooter.rev(() -> swerve.getDistance(targetPose))
                )
            ),
            this.feeder.intake(),
            this.shooter.stop()
        );
    }
}
