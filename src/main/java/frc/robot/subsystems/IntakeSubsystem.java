// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final TalonFX feederMotorLeft;
    private final TalonFX feederMotorRight;
    static boolean intaking = false;

    private final VelocityVoltage flControl = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityVoltage frControl = new VelocityVoltage(0).withEnableFOC(true);


    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(80);

        feederMotorLeft = new TalonFX(17, "rio");
        feederMotorRight = new TalonFX(18, "rio");

        applyConfigs();
    }

    private void applyConfigs() {
		var feederMotorConfig = new TalonFXConfiguration();
		feederMotorConfig.MotorOutput.NeutralMode = Constants.Shooter.motorNeutralValue;
		feederMotorConfig.MotorOutput.Inverted = Constants.Shooter.motorOutputInverted;
		feederMotorConfig.Voltage.PeakForwardVoltage = Constants.Shooter.peakForwardVoltage;
		feederMotorConfig.Voltage.PeakReverseVoltage = Constants.Shooter.peakReverseVoltage;

		feederMotorConfig.Slot0.kP = Constants.Shooter.kP;
		feederMotorConfig.Slot0.kI = Constants.Shooter.kI;
		feederMotorConfig.Slot0.kD = Constants.Shooter.kD;
		feederMotorConfig.Slot0.kS = Constants.Shooter.kS;
		feederMotorConfig.Slot0.kV = 1.0 / toRPS(Constants.Shooter.RPMsPerVolt);
		feederMotorConfig.Slot0.kA = 0.0;
		feederMotorConfig.Slot0.kG = 0.0;

		feederMotorLeft.getConfigurator().apply(feederMotorConfig);
		feederMotorRight.getConfigurator().apply(feederMotorConfig);
	}

    private double toRPS(double rpm) {
		return rpm / 60.0;
	}

    private void setFeederSpeed(double left, double right) {
		feederMotorLeft.setControl(flControl.withVelocity(toRPS(left)));
		feederMotorRight.setControl(frControl.withVelocity(toRPS(right)));
	}

    public void intake() {
        intakeMotor.set(Constants.Intake.intakingSpeed);
        setFeederSpeed(-500, -500);
    }
    public void Outtake(){
        intakeMotor.set(Constants.Intake.ejectingSpeed);
        setFeederSpeed(500, 500);
    }

    public void eject() {
        intakeMotor.set(Constants.Intake.ejectingSpeed);
        setFeederSpeed(100, 100);
    }

    /**
     * Stops both wheels.
     */
    public void stop() {
        intakeMotor.set(0);
        setFeederSpeed(0, 0);
    }

    /*@Override
    public void periodic() {
        double current = intakeMotor.getTorqueCurrent().getValueAsDouble();
        boolean active = (current > 20.0);

        if (active && !intaking) {
        }
        SmartDashboard.putNumber("intake/torqueCurrent", current);

    }*/
}