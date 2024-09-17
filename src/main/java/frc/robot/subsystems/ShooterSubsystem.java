import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private static TalonFX top;
    private static TalonFX bottom;

    public ShooterSubsystem() {
        top = new TalonFX(Constants.Shooter.topShooterID, "rio");
        bottom = new TalonFX(Constants.Shooter.bottomShooterID, "rio");
    }
}