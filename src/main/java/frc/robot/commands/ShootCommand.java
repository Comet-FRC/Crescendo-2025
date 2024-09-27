// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Speed;

public class ShootCommand extends Command {
	private final ShooterSubsystem shooter;
	private final FeederSubsystem feeder;

	private Speed speed;
	private boolean auton;

	public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, Speed speed) {
		this.shooter = shooter;
		this.feeder = feeder;
		this.speed = speed;
		addRequirements(shooter, feeder);
	}

	


	@Override
	public void initialize() {
		feeder.intake();
		shooter.shoot(speed);
		
	}

	@Override
	public void end(boolean interrupted) {
		shooter.stop();

		feeder.stop();
		
	}




}
