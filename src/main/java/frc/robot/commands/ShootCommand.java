// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Speed;

public class ShootCommand extends Command {
	private final ShooterSubsystem shooter;

	private Speed speed;

	public ShootCommand(ShooterSubsystem shooter, Speed speed) {
		this.shooter = shooter;
		this.speed = speed;
		addRequirements(shooter);
	}

	


	@Override
	public void initialize() {
		shooter.shoot(speed);
		
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();
	}




}
