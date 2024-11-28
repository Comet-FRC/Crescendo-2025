package org.cometrobotics.frc2024.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class AllianceColor {
  public static boolean isRed() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }
}
