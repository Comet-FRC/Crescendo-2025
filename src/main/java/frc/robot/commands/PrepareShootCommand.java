// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Speed;

public class PrepareShootCommand extends Command {
	private final ShooterSubsystem shooter;
	private final FeederSubsystem feeder;
	private final Speed speed;

	private final Timer indexTimer = new Timer();

	public PrepareShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, Speed speed) {
		this.shooter = shooter;
		this.feeder = feeder;
		addRequirements(shooter, feeder);
		this.speed = speed;
	}

	@Override
	public void initialize() {
		feeder.eject();
		shooter.shoot(speed);
		indexTimer.reset();
		indexTimer.start();
	}

	@Override
	public void execute() {
		if (indexTimer.hasElapsed(0.2))
			feeder.stop();
	}

	@Override
	public void end(boolean interrupted) {
		feeder.stop();
		shooter.stop();
	}

	@Override
	public boolean isFinished() {
		return shooter.isReady(true);
	}
}
