// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final double ROBOT_MASS = Units.lbsToKilograms(86.5);
	public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
	public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

	public static final class SWERVE {
		/**
		 * The maximum speed of the robot in meters per second, used to limit acceleration.
		 * 12.5 ft/s is the theoretical max speed of a MK4i L1 Neo-based drivetrain like
		 * the one we have.
		 */
		public static final double MAX_SPEED  = Units.feetToMeters(12.5);
	}


	public static final class AutonConstants {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0, 0);
		public static final PIDConstants ANGLE_PID       = new PIDConstants(4, 0, 0.01);
	}

	public static final class DrivebaseConstants {
		// Hold time on motor brakes when disabled
		public static final double WHEEL_LOCK_TIME = 10; // seconds
	}

	public static class OperatorConstants {
		/* Joystick Deadband */
		public static final double LEFT_X_DEADBAND  = 0.1;
		public static final double LEFT_Y_DEADBAND  = 0.1;
		public static final double RIGHT_X_DEADBAND = 0.1;
		public static final double TURN_CONSTANT    = 6;
	}

	public class Intake {
		/* IDs */
		public static final int motorID = 15;
		/* Motor Speed Values */
		public static final double intakingSpeed = -0.1;
		public static final double ejectingSpeed = 0.3;
		/* Motor Config Values */
		public static final double peakForwardVoltage = 12.0;
		public static final double peakReverseVoltage = -12.0;
		public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
		public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
	}

	public class Feeder {
		/* IDs */
		public static final int leftFeederID = 17;
		public static final int rightFeederID = 18;
	}

	public class Shooter {
		/* IDs */
		public static final int topShooterID = 13; 
		public static final int bottomShooterID = 14;
		/* Motor Speed Values */
		public static final double idleSpeed = 2500; 
		public static final double intakeSpeed = -800;
		public static final double stopSpeed = 0.00;
		public static final double topSpeed = 6000;
		public static final double maxRPMError = 60.0;
		public static final double maxRPMErrorLong = 30.0;
		public static final double slideShotVelocityErrorMax = 100.0;
		public static final double dumpShotVelocityErrorMax = 60.0;
		public static final double shuttleShotVelocityErrorMax = 75.0;
		public static final double farDistance = Units.inchesToMeters(114.0); // when more precision is required
		/* Motor Config Values */
		public static final double peakForwardVoltage = 12.0;
		public static final double peakReverseVoltage = -12.0;
		public static final InvertedValue motorOutputInverted = InvertedValue.Clockwise_Positive;
		public static final NeutralModeValue motorNeutralValue = NeutralModeValue.Brake;
		public static final double kP = 0.25; // Voltage per 1 RPS of error
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kS = 0.21;  // Voltage to overcome static friction
		public static final double RPMsPerVolt = 490;
		/* Time to complete shot once Note no longer detected */
		public static final double postShotTimeout = 0.1; // in seconds
	    }
}
