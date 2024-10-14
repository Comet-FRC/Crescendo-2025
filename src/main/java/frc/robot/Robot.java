// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;
import java.util.logging.Logger;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

	private static Robot instance;

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
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit()
	{
		m_robotContainer.getSwerveSubsystem().setMotorBrake(true);
		disabledTimer.reset();
		disabledTimer.start();
	}

	@Override
	public void disabledPeriodic()
	{
		if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
		{
			m_robotContainer.getSwerveSubsystem().setMotorBrake(false);
			disabledTimer.stop();
		}
	}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit()
	{
		m_robotContainer.getSwerveSubsystem().setMotorBrake(true);
		m_robotContainer.limelightShooter.updateDesiredDistance();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic()
	{
		m_robotContainer.updateRobotPose();
		m_robotContainer.updateAutonState();
		m_robotContainer.autonDrive(false);
	}

	@Override
	public void teleopInit()
	{
		m_robotContainer.limelightShooter.updateDesiredDistance();
		m_robotContainer.getSwerveSubsystem().setMotorBrake(true);
	}

	@Override
	public void teleopPeriodic()
	{
		m_robotContainer.updateRobotPose();
		m_robotContainer.updateState();
		m_robotContainer.drive(true);
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

	public static Logger getLogger() {
		return Logger.getLogger(Robot.class.getName());
	}

	public RobotContainer getRobotContainer() {
		return m_robotContainer;
	}
}
