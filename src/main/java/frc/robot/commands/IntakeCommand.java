// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
	private final IntakeSubsystem intake;

	public IntakeCommand(IntakeSubsystem intake) {
		this.intake = intake;
		addRequirements(this.intake);
	} 

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		intake.intake();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override	
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}