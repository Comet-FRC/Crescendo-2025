package org.cometrobotics.frc2024.robot.commands.shooter;

import org.cometrobotics.frc2024.robot.subsystems.FeederSubsystem;
import org.cometrobotics.frc2024.robot.subsystems.SwerveSubsystem;
import org.cometrobotics.frc2024.robot.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
                    //new IndexNote(),
                    this.shooter.setVelocityFromDistance(this.swerve::getDistanceFromSpeaker),
                    new WaitUntilCommand(this.shooter::isReady)
                )
            ),
            this.feeder.shoot(), // until there's no note
            this.feeder.stop(),
            this.shooter.stop()
        );

    }
}
