package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
private static TalonFX top;
private static TalonFX bottom;
private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);

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

	/**
	 * 
	 */
	public enum Speed {
		STOP,
		SUBWOOFER,
		AMP,
		SPEAKER
	}

	private final EnumMap<Speed, ShooterSpeed> shooterSpeeds = new EnumMap<>(Map.ofEntries(
		Map.entry(Speed.STOP, new ShooterSpeed(0, 0)),
		Map.entry(Speed.SPEAKER, new ShooterSpeed(3000, 3000)),
		Map.entry(Speed.AMP, new ShooterSpeed(650, 1150)),
		Map.entry(Speed.SUBWOOFER, new ShooterSpeed(1000, 6000))
	));

	public ShooterSubsystem() {
		top = new TalonFX(Constants.Shooter.topShooterID, "rio");
		bottom = new TalonFX(Constants.Shooter.bottomShooterID, "rio");
		applyConfigs();

		//SmartDashboard.putNumber("shooter/speed", 0.0);
		//SmartDashboard.putNumber("shooter/Top RPM adjustment", 0.0);
		//SmartDashboard.putNumber("shooter/Bottom RPM adjustment", 0.0);
	}

	private void applyConfigs() {
		var shooterMotorConfig = new TalonFXConfiguration();
		shooterMotorConfig.MotorOutput.NeutralMode = Constants.Shooter.motorNeutralValue;
		shooterMotorConfig.MotorOutput.Inverted = Constants.Shooter.motorOutputInverted;
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

	public void shoot(Speed speed) {
		setVelocity(speed);
	}

	private void setVelocity(Speed speed) {
		speedTarget = shooterSpeeds.get(speed);
		double topSpeed = speedTarget.topMotorSpeed;
		double bottomSpeed = speedTarget.bottomMotorSpeed;
		top.setControl(topControl.withVelocity(toRPS(topSpeed)));
		bottom.setControl(bottomControl.withVelocity(toRPS(bottomSpeed)));
	}

	public void stop() {
		setVelocity(Speed.STOP);
	}

	/**
	 * Returns true if the shooter has enough velocity to shoot.
	 * @param precise
	 * @return
	 */
	public boolean isReady(boolean precise) {
		boolean topIsReady = Math.abs(toRPM(top.getVelocity().getValueAsDouble()) - speedTarget.topMotorSpeed) < (precise ? Constants.Shooter.maxRPMErrorLong : Constants.Shooter.maxRPMError);
		boolean bottomIsReady = Math.abs(toRPM(bottom.getVelocity().getValueAsDouble()) - speedTarget.bottomMotorSpeed) < (precise ? Constants.Shooter.maxRPMErrorLong : Constants.Shooter.maxRPMError);
		return topIsReady && bottomIsReady;
	}
}
