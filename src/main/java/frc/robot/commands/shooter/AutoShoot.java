package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IndexNote;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoShoot extends SequentialCommandGroup {

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;

    public AutoShoot() {
        this.swerve = SwerveSubsystem.getInstance();
        this.shooter = ShooterSubsystem.getInstance();
        this.feeder = FeederSubsystem.getInstance();
        this.addRequirements(swerve, shooter, feeder);

        this.addCommands(
            new ParallelCommandGroup(
                this.swerve.turnToSpeaker(),
                new SequentialCommandGroup(
                    new IndexNote(),
                    this.shooter.setVelocityFromDistance(this.swerve::getDistanceFromSpeaker)
                        .until(this.shooter::isReady)
                )
            ),
            this.feeder.shoot(), // until there's no note
            this.feeder.stop(),
            this.shooter.stop()
        );

    }
}
