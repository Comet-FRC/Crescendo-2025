// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
	private final IntakeSubsystem intake;
	private final FeederSubsystem feeder;

	private final Timer timer = new Timer();
	boolean hasNote = false;

	private double intakingSpeed;

	public IntakeCommand(IntakeSubsystem intake, FeederSubsystem feeder) {
		this.intake = intake;
		this.feeder = feeder;
		addRequirements(intake, feeder);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		intakingSpeed = Constants.Intake.intakingSpeed;
		intake.intake();
		feeder.intake();
		hasNote = false;
		timer.reset();
		timer.stop();
	}

	@Override
	public void execute() {
		if (feeder.getTorqueCurrent() < -75) {
			hasNote = true;
			timer.start();
		}
		if (intakingSpeed > -1.0) {
			intakingSpeed -= 0.01;
			intake.set(intakingSpeed);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stop();
		feeder.stop();
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(0.5);
	}
}