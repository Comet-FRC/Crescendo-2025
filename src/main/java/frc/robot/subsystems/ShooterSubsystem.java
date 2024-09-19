package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
private static TalonFX top;
private static TalonFX bottom;
private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
private final VelocityVoltage topControl = new VelocityVoltage(0).withEnableFOC(true);
private final VelocityVoltage bottomControl = new VelocityVoltage(0).withEnableFOC(true);
private double topCurrentTarget = 0.0;
private double bottomCurrentTarget = 0.0;
private boolean autoAimingActive = false;

private class ShooterSpeed {
	double topMotorSpeed;
	double bottomMotorSpeed;

	public ShooterSpeed(double top, double bottom) {
	topMotorSpeed = top;
	bottomMotorSpeed = bottom;
	}
}

private final ShooterSpeed SUBWOOFER = new ShooterSpeed(1360, 2830);

public ShooterSubsystem() {
	top = new TalonFX(Constants.Shooter.topShooterID, "rio");
	bottom = new TalonFX(Constants.Shooter.bottomShooterID, "rio");
	applyConfigs();

	SmartDashboard.putNumber("shooter/Top RPM adjustment", 0.0);
	SmartDashboard.putNumber("shooter/Bottom RPM adjustment", 0.0);
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

	public void shoot() {
		setCurrentSpeed(SUBWOOFER);
	}



	private void setCurrentSpeed(ShooterSpeed speed) {
		topCurrentTarget = 1360;
		bottomCurrentTarget = 2830;
		top.setControl(topControl.withVelocity(toRPS(topCurrentTarget)));
		bottom.setControl(bottomControl.withVelocity(toRPS(bottomCurrentTarget)));
	}

	public void setVoltage(double voltage) {
		top.setControl(voltageOut.withOutput(voltage));
		bottom.setControl(voltageOut.withOutput(voltage));
}



	public void end() {
		topCurrentTarget = 0;
		bottomCurrentTarget = 0;
		top.setControl(topControl.withVelocity(toRPS(topCurrentTarget)));
		bottom.setControl(bottomControl.withVelocity(toRPS(bottomCurrentTarget)));
	}
}
