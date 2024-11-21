package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * @see https://github.com/Team-8-bit/2024-Sonic/blob/develop/robot/src/main/kotlin/org/team9432/robot/commands/drivetrain/TargetAim.kt
 */
public class AimCommand extends Command {
    private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private final Supplier<Translation2d> target;
    private ProfiledPIDController pid = new ProfiledPIDController(0.06, 0.0, 0.0, new TrapezoidProfile.Constraints(360.0, 360.0 * 2.0));

    public AimCommand(Supplier<Translation2d> target) {
        this.target = target;

        this.pid.enableContinuousInput(-180, 180);
        this.pid.setTolerance(3.0);
        //TODO: look into if setting 

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pid.reset(swerve.getYaw().getDegrees());
    }

    @Override
    public void execute() {
        Translation2d currentTarget = target.get();
        this.pid.setGoal(swerve.getAngleTo(currentTarget).getDegrees());

        double angularVelocity = pid.calculate(swerve.getPose().getRotation().getDegrees());
        ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(0,0, angularVelocity, swerve.getYaw());

        this.swerve.drive(speed);
    }

    @Override
    public boolean isFinished() {
        return this.pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.swerve.stop();
    }
}
