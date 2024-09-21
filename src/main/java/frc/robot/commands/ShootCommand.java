// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootCommand extends Command {
	private final ShooterSubsystem shooter;
	private boolean cancelled = false;
	private Timer postShotTimer = new Timer();
	private boolean gone = false;

	public ShootCommand(ShooterSubsystem shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	// Overloaded constructor with Swerve for autonomous movement
	public ShootCommand(ShooterSubsystem shooter, SwerveSubsystem swerve) {
		this(shooter);
	}

	@Override
	public void initialize() {
		cancelled = false;
		gone = false;

		shooter.shoot();
	}

	@Override
	public void execute() {
		if (cancelled) return;
	}


	@Override
	public void end(boolean interrupted) {
		shooter.end();
	}

	@Override
	public boolean isFinished() {
		return cancelled || (gone && postShotTimer.hasElapsed(Constants.Shooter.postShotTimeout));
	}
}
