package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class AllianceColor {
  public static boolean isRed() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }
}
