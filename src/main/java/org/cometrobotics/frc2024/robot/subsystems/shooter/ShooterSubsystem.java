package org.cometrobotics.frc2024.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.cometrobotics.frc2024.robot.Constants;
import org.cometrobotics.frc2024.robot.RobotContainer;
import org.cometrobotics.frc2024.robot.RobotContainer.State;
import org.cometrobotics.frc2024.robot.subsystems.misc.ProximitySensor;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterSubsystem extends SubsystemBase {
	/* Singleton */
	
	private static ShooterSubsystem instance = null;
	
	public static ShooterSubsystem getInstance() {
		if (instance == null) instance = new ShooterSubsystem();
		return instance;
	}

	/* Implementation */

	private final TalonFX top;
	private final TalonFX bottom;

	private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
	private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);

	private final RangeTable RANGE_TABLE = new RangeTable();

	private ShooterSubsystem() {
		this.top = new TalonFX(Constants.SHOOTER.topShooterID, "rio");
		this.bottom = new TalonFX(Constants.SHOOTER.bottomShooterID, "rio");
		this.setupMotors();
	}

	private void setupMotors() {
		var shooterMotorConfig = new TalonFXConfiguration();
		shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		shooterMotorConfig.Voltage.PeakForwardVoltage = Constants.SHOOTER.peakForwardVoltage;
		shooterMotorConfig.Voltage.PeakReverseVoltage = Constants.SHOOTER.peakReverseVoltage;

		shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		shooterMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
		shooterMotorConfig.CurrentLimits.SupplyTimeThreshold = 0.5;

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

	public Command setVelocityFromDistance(DoubleSupplier distance) {
		return 
			Commands.runOnce(
				() -> {
					RobotContainer.setState(State.REVVING);
				},
				this
			).andThen(this.setVelocity(() -> RANGE_TABLE.get(distance.getAsDouble())));
	}

	public Command setVelocity(Supplier<ShooterSpeed> speed) {
		return 
			Commands.runOnce(() -> {
				double topSpeed = speed.get().topMotorSpeed;
				double bottomSpeed = speed.get().bottomMotorSpeed;
				this.top.setControl(this.topControl.withVelocity(this.toRPS(topSpeed)));
				this.bottom.setControl(this.bottomControl.withVelocity(this.toRPS(bottomSpeed)));
			}, this);
	}

	public Command shoot() {
		
		SmartDashboard.putNumber("shooter/topMotorSpeed", 0);
		SmartDashboard.putNumber("shooter/bottomMotorSpeed", 0);

		return
			this.setVelocity(
				() -> {
					double topShooterSpeed = SmartDashboard.getNumber("shooter/topMotorSpeed", 0);
					double bottomShooterSpeed = SmartDashboard.getNumber("shooter/bottomMotorSpeed", 0);
					return new ShooterSpeed(topShooterSpeed,bottomShooterSpeed);
				}
			)
			.andThen(
				new WaitUntilCommand(() -> !ProximitySensor.getInstance().hasObject())
			);
	}

	public Command eject() {
		return this.setVelocity(() -> new ShooterSpeed(-500, -500));
	}

	public Command stop() {
		return this.setVelocity(() -> new ShooterSpeed(0, 0));
	}

	/**
	 * Returns true if the shooter has enough velocity to shoot.
	 * @param precise
	 * @return
	 */
	public boolean isReady() {
		boolean topIsReady = Math.abs(toRPM(top.getClosedLoopError().getValueAsDouble())) < Constants.SHOOTER.maxRPMError;
		boolean bottomIsReady = Math.abs(toRPM(bottom.getClosedLoopError().getValueAsDouble())) < Constants.SHOOTER.maxRPMError;

		return topIsReady && bottomIsReady;
	}

	@Override
	public void periodic() {

		double targetTopMotorSpeed = toRPM(top.getClosedLoopReference().getValueAsDouble());
		double measuredTopMotorSpeed = toRPM(top.getVelocity().getValueAsDouble());
		double topSpeedError = toRPM(top.getClosedLoopError().getValueAsDouble());

		Logger.recordOutput("Robot/Shooter/Top/Target Speed", targetTopMotorSpeed);
		Logger.recordOutput("Robot/Shooter/Top/Measured Speed", measuredTopMotorSpeed);
		Logger.recordOutput("Robot/Shooter/Top/Speed Error", topSpeedError);
		
		double targetBottomMotorSpeed = toRPM(bottom.getClosedLoopReference().getValueAsDouble());
		double measuredBottomMotorSpeed = toRPM(bottom.getVelocity().getValueAsDouble());
		double bottomSpeedError = toRPM(bottom.getClosedLoopError().getValueAsDouble());

		Logger.recordOutput("Robot/Shooter/Bottom/Target Speed", targetBottomMotorSpeed);
		Logger.recordOutput("Robot/Shooter/Bottom/Measured Speed", measuredBottomMotorSpeed);
		Logger.recordOutput("Robot/Shooter/Bottom/Speed Error", bottomSpeedError);
	}
}
