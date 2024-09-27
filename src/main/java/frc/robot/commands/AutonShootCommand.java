// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Speed;

public class AutonShootCommand extends Command {
	private final ShooterSubsystem shooter;
	private final FeederSubsystem feeder;
	private final IntakeSubsystem intake;
	private final Speed speed;

	public AutonShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, IntakeSubsystem intake, Speed speed) {
		this.shooter = shooter;
		this.feeder = feeder;
		this.intake = intake;
		addRequirements(shooter, feeder);
		this.speed = speed;
	}

	@Override
	public void initialize() {
		feeder.intake();
		intake.intake();
		shooter.shoot(speed);
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
		feeder.stop();
		shooter.stop();
	}
}

