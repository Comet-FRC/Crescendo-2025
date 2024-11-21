// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

	/* Singleton */
	
	private static IntakeSubsystem instance = null;
	
	public static IntakeSubsystem getInstance() {
		if (instance == null) instance = new IntakeSubsystem();
		return instance;
	}

	/* Implementation */

    private final CANSparkMax intakeMotor;
    static boolean intaking = false;

    double speed = 0;

    private IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(80);
    }

    public void set(double speed) {
        // Only set the speed if it's not already the speed.
        if (speed == this.speed) {
            return;
        }
        
        intakeMotor.set(speed);
        this.speed = speed;
    }

    public void intake() {
        set(Constants.Intake.intakingSpeed);
    }

    public void eject() {
        set(Constants.Intake.ejectingSpeed);
    }

    /**
     * Stops both wheels.
     */
    public void stop() {
        set(0);
    }
}