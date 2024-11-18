package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
	private static TalonFX top;
	private static TalonFX bottom;
	private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
	private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);

	private final Timer motorTimer = new Timer();

	private ShooterSpeed speedTarget = new ShooterSpeed(0, 0);

	/**
	 * Represents the speeds of the top and bottom motors of the shooter.
	 */
	public class ShooterSpeed {
		double topMotorSpeed;
		double bottomMotorSpeed;

		public ShooterSpeed(double top, double bottom) {
			topMotorSpeed = top;
			bottomMotorSpeed = bottom;
		}
	}

	public class ShootReference {
		private Pose2d pose;
		private ShooterSpeed shooterSpeed;

		ShootReference(
			Pose2d fieldRelativePose,
			ShooterSpeed shooterSpeed
		) {
			pose = fieldRelativePose;
			this.shooterSpeed = shooterSpeed;
		}

		ShootReference(
			double xPos,
			double yPos,
			double rot,
			ShooterSpeed shooterSpeed
					) {
			pose = new Pose2d(new Translation2d(xPos, yPos), new Rotation2d(rot));
			this.shooterSpeed = shooterSpeed;
		}

		public ShootReference() {
            pose = new Pose2d();
			this.shooterSpeed = new ShooterSpeed(0, 0);
        }

        public Pose2d getPose() {
			return pose;
		}

		public ShooterSpeed getShooterSpeed() {
			return shooterSpeed;
		}

		public double calculateDistance(Pose2d o) {
			return pose.getTranslation().getDistance(o.getTranslation());
		}
	}

	final ShootReference[] SPEAKER_RANGE_TABLE = {
		new ShootReference(1.831, 5.494, 180, new ShooterSpeed(3100, 3100))
	};

	public ShootReference getClosestShootReference(Pose2d robotPose) {
		ShootReference closestShootReference = null;
		double closestDistance = Double.MAX_VALUE;

		for (int i=0; i<SPEAKER_RANGE_TABLE.length; ++i) {
			double newDistance = SPEAKER_RANGE_TABLE[i].calculateDistance(robotPose);
			if (newDistance < closestDistance) {
				closestDistance = newDistance;
				closestShootReference = SPEAKER_RANGE_TABLE[i];
			}
		}

		return closestShootReference;
	}

	public ShooterSubsystem() {
		top = new TalonFX(Constants.Shooter.topShooterID, "rio");
		bottom = new TalonFX(Constants.Shooter.bottomShooterID, "rio");
		applyConfigs();
	}

	private void applyConfigs() {
		var shooterMotorConfig = new TalonFXConfiguration();
		shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		shooterMotorConfig.Voltage.PeakForwardVoltage = Constants.Shooter.peakForwardVoltage;
		shooterMotorConfig.Voltage.PeakReverseVoltage = Constants.Shooter.peakReverseVoltage;

		shooterMotorConfig.Slot0.kP = Constants.Shooter.kP;
		shooterMotorConfig.Slot0.kI = Constants.Shooter.kI;
		shooterMotorConfig.Slot0.kD = Constants.Shooter.kD;
		shooterMotorConfig.Slot0.kS = Constants.Shooter.kS;
		shooterMotorConfig.Slot0.kV = 1.0 / toRPS(Constants.Shooter.RPMsPerVolt);
		shooterMotorConfig.Slot0.kA = 0.0;
		shooterMotorConfig.Slot0.kG = 0.0;

		top.getConfigurator().apply(shooterMotorConfig);
		bottom.getConfigurator().apply(shooterMotorConfig);
	}

	private double toRPM(double rps) {
		return rps * 60.0;
	}

	private double toRPS(double rpm) {
		return rpm / 60.0;
	}

	public void setVelocity(ShooterSpeed speed) {
		// The target speed is already set to that value, no action needed
		if (speedTarget.equals(speed)) {
			return;
		}
		motorTimer.restart();
		speedTarget = speed;
		double topSpeed = speedTarget.topMotorSpeed;
		double bottomSpeed = speedTarget.bottomMotorSpeed;
		top.setControl(topControl.withVelocity(toRPS(topSpeed)));
		bottom.setControl(bottomControl.withVelocity(toRPS(bottomSpeed)));
	}

	public void eject() {
		setVelocity(new ShooterSpeed(-500, -500));
	}

	public void stop() {
		setVelocity(new ShooterSpeed(0, 0));
	}

	/**
	 * Returns true if the shooter has enough velocity to shoot.
	 * @param precise
	 * @return
	 */
	public boolean isReady(boolean precise) {
		if (motorTimer.hasElapsed(1))
			return true;

		boolean topIsReady = Math.abs(toRPM(top.getVelocity().getValueAsDouble()) - speedTarget.topMotorSpeed) < (precise ? Constants.Shooter.maxRPMErrorLong : Constants.Shooter.maxRPMError);
		boolean bottomIsReady = Math.abs(toRPM(bottom.getVelocity().getValueAsDouble()) - speedTarget.bottomMotorSpeed) < (precise ? Constants.Shooter.maxRPMErrorLong : Constants.Shooter.maxRPMError);

		return topIsReady && bottomIsReady;
	}
}
