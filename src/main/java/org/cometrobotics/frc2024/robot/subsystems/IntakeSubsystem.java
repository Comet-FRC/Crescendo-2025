// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.cometrobotics.frc2024.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.cometrobotics.frc2024.robot.Constants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeSubsystem extends SubsystemBase {

	/* Singleton */
	
	private static IntakeSubsystem instance = null;
	
	public static IntakeSubsystem getInstance() {
		if (instance == null) instance = new IntakeSubsystem();
		return instance;
	}

	/* Implementation */

    private final CANSparkMax intakeMotor;
    private final SparkPIDController pid;
    private final SimpleMotorFeedforward ff;
    
    private double targetSpeed = 0;

    double speed = 0;

    private IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.INTAKE.motorID, MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();

        this.pid = intakeMotor.getPIDController();

        // TODO: test new values
        pid.setP(4.6803E-06);
        pid.setI(0);
        pid.setD(0);

        this.ff = new SimpleMotorFeedforward(0.011483, 0.0077225, 0.0018332);

        intakeMotor.setSmartCurrentLimit(80);
    }

    public Command setVelocity(DoubleSupplier speed) {
        return Commands.run(() -> {
            this.pid.setReference(
                speed.getAsDouble(),
                ControlType.kVelocity,
                0,
                ff.calculate(intakeMotor.getEncoder().getVelocity())
            );
            this.targetSpeed = speed.getAsDouble();
        },
        this
        );
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

    @Override
    public void periodic() {
		Logger.recordOutput("Robot/Intake/Target Speed", targetSpeed);
		Logger.recordOutput("Robot/Feeder/Measured Speed", intakeMotor.getEncoder().getVelocity());
    }

    public Command sysId() {
        SysIdRoutine routine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,               // Use default ramp rate (1 V/s)
                    Units.Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                    null                // Use default timeout (10 s)
                ),
                new SysIdRoutine.Mechanism(
                    (voltage) -> {
                    intakeMotor.setVoltage(voltage.in(Units.Volts));
                    },
                    (logger) -> {
                    logger
                        .motor("left")
                        .voltage(
                            Units.Volts.of(
                                intakeMotor.getBusVoltage()
                                    * intakeMotor.getAppliedOutput()))
                        .linearPosition(Units.Meters.of(intakeMotor.getEncoder().getPosition()))
                        .linearVelocity(Units.MetersPerSecond.of(intakeMotor.getEncoder().getVelocity()));
                    },
                    this));

        return routine
            .dynamic(SysIdRoutine.Direction.kForward)
            .andThen(Commands.waitSeconds(3))
            .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse))
            .andThen(Commands.waitSeconds(3))
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kForward))
            .andThen(Commands.waitSeconds(3))
            .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse));
    }
}