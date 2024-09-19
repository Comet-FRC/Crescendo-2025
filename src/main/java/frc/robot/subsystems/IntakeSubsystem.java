// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    static boolean intaking = false;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(80);
    }

    public void intake() {
        intakeMotor.set(Constants.Intake.intakingSpeed);
    }

    public void eject() {
        intakeMotor.set(Constants.Intake.ejectingSpeed);
    }

    /**
     * Stops both wheels.
     */
    public void stop() {
        intakeMotor.set(0);
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