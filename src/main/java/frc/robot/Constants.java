// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class AUTON {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(16, 0, 0);
		public static final PIDConstants ANGLE_PID       = new PIDConstants(4, 0, 0);
	}

	public class VISION {
		public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEV = VecBuilder.fill(.7,.7,9999999);
	}

	public class SWERVE {
		/**
		 * The maximum speed of the robot in meters per second, used to limit acceleration.
		 * 12.5 ft/s is the theoretical max speed of a MK4i L1 Neo-based drivetrain like
		 * the one we have.
		 */
		public final static double MAX_SPEED  = Units.feetToMeters(12.5);
		// Hold time on motor brakes when disabled
		public final static double WHEEL_LOCK_TIME = 10; // seconds
	}

	public class INTAKE {
		/* IDs */
		public static final int motorID = 15;
		/* Motor Speed Values */
		public static final double intakingSpeed = 0.5;
		public static final double shootingSpeed = 0.5;
		public static final double ejectingSpeed = -0.5;
		/* Motor Config Values */
		public static final double peakForwardVoltage = 12.0;
		public static final double peakReverseVoltage = -12.0;
	}

	public class FEEDER {
		/* IDs */
		public static final int leftFeederID = 17;
		public static final int rightFeederID = 18;
		public static final int laserCanID = 19;
	}

	public class SHOOTER {
		/* IDs */
		public static final int topShooterID = 13; 
		public static final int bottomShooterID = 14;
		/* Motor Speed Values */
		public static final double maxRPMError = 60.0;
		/* Motor Config Values */
		public static final double peakForwardVoltage = 12.0;
		public static final double peakReverseVoltage = -12.0;
		public static final double kP = 0.25; // Voltage per 1 RPS of error
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kS = 0.21;  // Voltage to overcome static friction
		public static final double RPMsPerVolt = 490;
		/* Time to complete shot once Note no longer detected */
		public static final double postShotTimeout = 0.5; // in seconds
	    }
}
