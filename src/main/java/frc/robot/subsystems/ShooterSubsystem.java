package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
	/* Singleton */
	
	private static ShooterSubsystem instance = null;
	
	public static ShooterSubsystem getInstance() {
		if (instance == null) instance = new ShooterSubsystem();
		return instance;
	}

	/* Implementation */

	private static TalonFX top;
	private static TalonFX bottom;

	private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
	private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);

	private ShooterSpeed speedTarget = new ShooterSpeed(0, 0);

	private final InterpolatingTreeMap<Double, ShooterSpeed> RANGE_TABLE_SPEAKER;

	/**
	 * Represents the speeds of the top and bottom motors of the shooter.
	 */
	public static class ShooterSpeed implements Interpolatable<ShooterSpeed> {
		double topMotorSpeed;
		double bottomMotorSpeed;

		public ShooterSpeed(double top, double bottom) {
			topMotorSpeed = top;
			bottomMotorSpeed = bottom;
		}

		public ShooterSpeed diff (ShooterSpeed o) {
			return new ShooterSpeed(
				topMotorSpeed - o.topMotorSpeed,
				bottomMotorSpeed - o.bottomMotorSpeed
			);
		}

		public ShooterSpeed sum(ShooterSpeed o) {
			return new ShooterSpeed(
				topMotorSpeed + o.topMotorSpeed,
				bottomMotorSpeed + o.bottomMotorSpeed
			);
		}

		public ShooterSpeed product(double scalar) {
			return new ShooterSpeed(
				topMotorSpeed * scalar,
				bottomMotorSpeed * scalar
			);
		}

		@Override
		public ShooterSpeed interpolate(ShooterSpeed endValue, double t) {
			
			ShooterSpeed delta = this.diff(endValue);

			ShooterSpeed newSpeed = this.sum(delta.product(t));
			return newSpeed;
		}

		/**
		 * Returns interpolator for Double.
		 *
		 * @return Interpolator for Double.
		 */
		static Interpolator<ShooterSpeed> getInterpolator() {
			return ShooterSpeed::interpolate;
		}
	}

	private ShooterSubsystem() {
		top = new TalonFX(Constants.SHOOTER.topShooterID, "rio");
		bottom = new TalonFX(Constants.SHOOTER.bottomShooterID, "rio");
		applyConfigs();

		// For setting the range table
		SmartDashboard.putNumber("robot/shooter/topSpeed", 2000);
		SmartDashboard.putNumber("robot/shooter/bottomSpeed", 2000);

		RANGE_TABLE_SPEAKER = new InterpolatingTreeMap<Double, ShooterSpeed>(
            InverseInterpolator.forDouble(),
			ShooterSpeed.getInterpolator()
        );

		RANGE_TABLE_SPEAKER.put(0.91186, new ShooterSpeed(1200, 3200));
		RANGE_TABLE_SPEAKER.put(1.19126, new ShooterSpeed(1500, 2500));
		RANGE_TABLE_SPEAKER.put(1.5113, new ShooterSpeed(2200, 2200));
		RANGE_TABLE_SPEAKER.put(1.83388, new ShooterSpeed(2700, 2200));
		RANGE_TABLE_SPEAKER.put(2.12089, new ShooterSpeed(3000, 1900));
		RANGE_TABLE_SPEAKER.put(2.43178, new ShooterSpeed(2900, 1700));
		RANGE_TABLE_SPEAKER.put(2.74574, new ShooterSpeed(2800, 1550));
		RANGE_TABLE_SPEAKER.put(3.07186, new ShooterSpeed(2800, 1425));
		RANGE_TABLE_SPEAKER.put(3.35328, new ShooterSpeed(2700, 1400));

		// TODO: MEASURE RANGE TABLE VALUES
		// THIS IS THE ONLY ONE WE KNOW FOR SURE WORKS RN
        RANGE_TABLE_SPEAKER.put(2.34, new ShooterSpeed(3100, 3100));
	}

	private void applyConfigs() {
		var shooterMotorConfig = new TalonFXConfiguration();
		shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		shooterMotorConfig.Voltage.PeakForwardVoltage = Constants.SHOOTER.peakForwardVoltage;
		shooterMotorConfig.Voltage.PeakReverseVoltage = Constants.SHOOTER.peakReverseVoltage;

		shooterMotorConfig.Slot0.kP = Constants.SHOOTER.kP;
		shooterMotorConfig.Slot0.kI = Constants.SHOOTER.kI;
		shooterMotorConfig.Slot0.kD = Constants.SHOOTER.kD;
		shooterMotorConfig.Slot0.kS = Constants.SHOOTER.kS;
		shooterMotorConfig.Slot0.kV = 1.0 / toRPS(Constants.SHOOTER.RPMsPerVolt);
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

	public Command revSpeaker(DoubleSupplier distance) {
		return new InstantCommand(() -> {
			setVelocity(RANGE_TABLE_SPEAKER.get(distance.getAsDouble()));
			Logger.recordOutput("shooter/rev speaker distance", distance.getAsDouble());
		}, this)
			.andThen(new WaitUntilCommand(this::isReady));
	}

	public void setVelocity(ShooterSpeed speed) {
		// The target speed is already set to that value, no action needed
		if (speedTarget.equals(speed)) {
			return;
		}
		speedTarget = speed;
		double topSpeed = speedTarget.topMotorSpeed;
		double bottomSpeed = speedTarget.bottomMotorSpeed;
		top.setControl(topControl.withVelocity(toRPS(topSpeed)));
		bottom.setControl(bottomControl.withVelocity(toRPS(bottomSpeed)));
	}

	public void eject() {
		setVelocity(new ShooterSpeed(-500, -500));
	}

	public Command stop() {
		return new InstantCommand(() -> setVelocity(new ShooterSpeed(0, 0)));
	}

	/**
	 * Returns true if the shooter has enough velocity to shoot.
	 * @param precise
	 * @return
	 */
	public boolean isReady() {

		boolean topIsReady = Math.abs(toRPM(top.getVelocity().getValueAsDouble()) - speedTarget.topMotorSpeed) < (Constants.SHOOTER.maxRPMError);
		boolean bottomIsReady = Math.abs(toRPM(bottom.getVelocity().getValueAsDouble()) - speedTarget.bottomMotorSpeed) < (Constants.SHOOTER.maxRPMError);

		return topIsReady && bottomIsReady;
	}
}
