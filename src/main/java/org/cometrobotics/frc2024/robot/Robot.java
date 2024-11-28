// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.cometrobotics.frc2024.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.File;
import java.io.IOException;

import org.cometrobotics.frc2024.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot
{

	private static Robot instance;
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	private Timer disabledTimer;

	public Robot() {
		instance = this;
	}

	public static Robot getInstance() {
		return instance;
	}

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

		if (isReal()) {
			//Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
		} else if (isSimulation()) {
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
		} else {
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
		}

		// Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
		Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();

		// Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
		// immediately when disabled, but then also let it be pushed more 
		disabledTimer = new Timer();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
	 * during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
		Logger.recordOutput("Match Info/Match Time", DriverStation.getMatchTime());
		Logger.recordOutput("Robot/Battery Voltage", RobotController.getBatteryVoltage());
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit()
	{
		SwerveSubsystem.getInstance().setMotorBrake(true);
		disabledTimer.reset();
		disabledTimer.start();
	}

	@Override
	public void disabledPeriodic()
	{
		if (disabledTimer.hasElapsed(Constants.SWERVE.WHEEL_LOCK_TIME))
		{
			SwerveSubsystem.getInstance().setMotorBrake(false);
			disabledTimer.stop();
		}
	}

	@Override
	public void autonomousInit()
	{
		SwerveSubsystem.getInstance().setMotorBrake(true);

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		// schedule the autonomous command (example)
		if (m_autonomousCommand != null)
		{
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic()
	{
		m_robotContainer.updatePoseEstimation();
	}

	@Override
	public void teleopInit()
	{
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null)
		{
			m_autonomousCommand.cancel();
		}
		CommandScheduler.getInstance().cancelAll();
		SwerveSubsystem.getInstance().setMotorBrake(true);
	}

	@Override
	public void teleopPeriodic()
	{
		m_robotContainer.updatePoseEstimation();
	}

	@Override
	public void testInit()
	{
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
		try
		{
			new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
		} catch (IOException e)
		{
			throw new RuntimeException(e);
		}
	}

	@Override
	public void simulationInit() {
		DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
	}

	@Override
	public void simulationPeriodic() {
	}

	public RobotContainer getRobotContainer() {
		return m_robotContainer;
	}
}
