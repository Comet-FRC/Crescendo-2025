// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.cometrobotics.frc2024.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.cometrobotics.frc2024.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

	/* Singleton */
	
	private static IntakeSubsystem instance = null;
	
	public static IntakeSubsystem getInstance() {
		if (instance == null) instance = new IntakeSubsystem();
		return instance;
	}

	/* Implementation */

    private final CANSparkMax intakeMotor;

    double speed = 0;

    private IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.INTAKE.motorID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(80);
    }

    public Command setVelocity(DoubleSupplier speed) {
        return Commands.runOnce(() -> this.intakeMotor.set(speed.getAsDouble()), this);
    }

    public Command intake() {
        return this.setVelocity(() -> Constants.INTAKE.intakingSpeed);
    }

    public Command shoot() {
        return this.setVelocity(() -> Constants.INTAKE.shootingSpeed);
    }

    public Command outtake() {
        return this.setVelocity(() -> Constants.INTAKE.ejectingSpeed);
    }

    public Command stop() {
        return this.setVelocity(() -> 0);
    }
}