// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.cometrobotics.frc2024.robot.subsystems;

import org.cometrobotics.frc2024.robot.Constants;
import org.cometrobotics.frc2024.robot.subsystems.misc.ProximitySensor;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class FeederSubsystem extends SubsystemBase {

	/* Singleton */
	
	private static FeederSubsystem instance = null;
	
	public static FeederSubsystem getInstance() {
		if (instance == null) instance = new FeederSubsystem();
		return instance;
	}

	/* Implementation */

    private TalonFX feederMotorLeft;
    private TalonFX feederMotorRight;
    private final VelocityVoltage flControl;
    private final VelocityVoltage frControl;

    private FeederSubsystem() {
        this.feederMotorLeft = new TalonFX(Constants.FEEDER.leftFeederID, "rio");
        this.feederMotorRight = new TalonFX(Constants.FEEDER.rightFeederID, "rio");
        this.flControl = new VelocityVoltage(0).withEnableFOC(true);
        this.frControl = new VelocityVoltage(0).withEnableFOC(true);

        applyConfigs();
    }

    private void applyConfigs() {
		TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration();
		feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		feederMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		feederMotorConfig.Voltage.PeakForwardVoltage = Constants.INTAKE.peakForwardVoltage;
		feederMotorConfig.Voltage.PeakReverseVoltage = Constants.INTAKE.peakReverseVoltage;
        
		feederMotorConfig.Slot0.kP = Constants.SHOOTER.kP;
		feederMotorConfig.Slot0.kI = Constants.SHOOTER.kI;
		feederMotorConfig.Slot0.kD = Constants.SHOOTER.kD;
		feederMotorConfig.Slot0.kS = Constants.SHOOTER.kS;
		feederMotorConfig.Slot0.kV = 1.0 / toRPS(Constants.SHOOTER.RPMsPerVolt);
		feederMotorConfig.Slot0.kA = 0.0;
		feederMotorConfig.Slot0.kG = 0.0;
        
		this.feederMotorLeft.getConfigurator().apply(feederMotorConfig);

        feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		this.feederMotorRight.getConfigurator().apply(feederMotorConfig);
	}

    public void setVelocity(double speedRPM) {
		this.feederMotorLeft.setControl(flControl.withVelocity(toRPS(speedRPM)));
		this.feederMotorRight.setControl(frControl.withVelocity(toRPS(speedRPM)));
	}

    private double toRPS(double rpm) {
		return rpm / 60.0;
	}

    public Command intake() {
        return Commands.runOnce(() -> this.setVelocity(900), this)
            .until(() -> (ProximitySensor.getInstance().hasObject() || this.getTorqueCurrent() < -25));
    }

    public Command shoot() {
        return Commands.run(() -> this.setVelocity(4000), this)
            .until(() -> !ProximitySensor.getInstance().hasObject());
    }

    public Command outtake() {
        return Commands.runOnce(() -> this.setVelocity(-500), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> this.setVelocity(0), this);
    }

    public double getTorqueCurrent() {
        double sum = 0;
        sum += feederMotorLeft.getTorqueCurrent().getValueAsDouble();
        sum += feederMotorRight.getTorqueCurrent().getValueAsDouble();
        sum /= 2.0;

        return sum;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Robot/Feeder/Proximity Distance mm", ProximitySensor.getInstance().getDistanceMM());
    }




    /* SYS ID */

    private final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,               // Use default ramp rate (1 V/s)
                Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,                // Use default timeout (10 s)
                                             // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> feederMotorLeft.setControl(new VoltageOut(volts.in(Units.Volts))),
                null,
                this
            )
        );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
        
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}