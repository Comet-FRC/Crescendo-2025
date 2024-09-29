// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.AutonPrepareShootCommand;
import frc.robot.commands.AutonShootCommand;
import frc.robot.commands.EjectCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Speed;

import java.io.File;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	/* Autonomous */
	private final SendableChooser<Command> autoChooser;
	
	/* Controllers */
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final Joystick operatorController = new Joystick(1);
	
	/* Buttons */
	private final Trigger zeroGyroButton = driverController.a();

	private final Trigger ampButton = new JoystickButton(operatorController, 4);
	private final Trigger intakeButton = new JoystickButton(operatorController, 5);
	private final Trigger shooterButton = new JoystickButton(operatorController, 6);
	private final Trigger ejectButton = new JoystickButton(operatorController, 7);
	/* Subsystems */
	private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
	private final IntakeSubsystem intake = new IntakeSubsystem();
	private final FeederSubsystem feeder = new FeederSubsystem();
	private final ShooterSubsystem shooter = new ShooterSubsystem();
	private final VisionSubsystem vision = new VisionSubsystem(swerve, "limelight-shooter");
	
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		DriverStation.silenceJoystickConnectionWarning(true);

		/* REGISTERING COMMANDS FOR PATHPLANNER */
		NamedCommands.registerCommand("SubWoof",
			Commands.runOnce(() -> {SmartDashboard.putString("Auto Status", "Begin SW shot");})
				.andThen(new AutonPrepareShootCommand(shooter, Speed.SPEAKER))
				.andThen(new AutonShootCommand(shooter, feeder, intake, Speed.SPEAKER)).withTimeout(2)
				
				.andThen(Commands.runOnce(() -> {  SmartDashboard.putString("Auto Status", "SW complete"); }))
			);
		configureAutonCommands();
		/* Configure the trigger bindings */
		configureBindings();

		autoChooser = AutoBuilder.buildAutoChooser();
			SmartDashboard.putData("auto/Auto Chooser", autoChooser);
	}

	private void configureAutonCommands() {
		/* Subwoofer Shot */
		
		
		/* Intake */
		NamedCommands.registerCommand("Intake note",
			Commands.runOnce(() -> { 
				SmartDashboard.putString("Auto Status", "Beginning Intake");
			})
			.andThen(new IntakeCommand(intake, feeder))
			.andThen(Commands.runOnce(() -> { 
				SmartDashboard.putString("Auto Status", "Intake Complete");
			}))
		);
	}


	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
	 * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
	 */
	private void configureBindings() {
		zeroGyroButton.onTrue((Commands.runOnce(swerve::zeroGyro)));

		ampButton.whileTrue(new ShootCommand(shooter, Speed.AMP));

		ejectButton.whileTrue(new EjectCommand(intake, feeder).withName("Eject"));
		
		shooterButton.whileTrue(
			//new PrepareShootCommand(shooter, feeder, Speed.SPEAKER)
			/* .andThen(*/new ShootCommand(shooter, Speed.SPEAKER)
			.withName("Shoot Command"));
	
		
		
		intakeButton.whileTrue(new IntakeCommand(intake, feeder).withName("Intake"));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}

	public void setDriveMode() {
		//drivebase.setDefaultCommand();
	}

	public void setMotorBrake(boolean brake) {
		swerve.setMotorBrake(brake);
	}

	
	public Joystick getOperatorController () {
		return operatorController;
	}

	// simple proportional turning control with Limelight.
	// "proportional control" is a control algorithm in which the output is proportional to the error.
	// in this case, we are going to return an angular velocity that is proportional to the 
	// "tx" value from the Limelight.
	double limelight_aim_proportional()
	{    
		// kP (constant of proportionality)
		// this is a hand-tuned number that determines the aggressiveness of our proportional control loop
		// if it is too high, the robot will oscillate.
		// if it is too low, the robot will never reach its target
		// if the robot never turns in the correct direction, kP should be inverted.
		double kP = .035;

		// tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
		// your limelight 3 feed, tx should return roughly 31 degrees.
		double targetingAngularVelocity = LimelightHelpers.getTX("limelight-shooter") * kP;

		// convert to radians per second for our drive method
		targetingAngularVelocity *= swerve.getMaximumAngularVelocity();

		//invert since tx is positive when the target is to the right of the crosshair
		targetingAngularVelocity *= -1.0;

		return targetingAngularVelocity;
	}

	// simple proportional ranging control with Limelight's "ty" value
	// this works best if your Limelight's mount height and target mount height are different.
	// if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
	double limelight_range_proportional()
	{    
		double kP = .1;
		double targetingForwardSpeed = LimelightHelpers.getTY("limelight-shooter") * kP;
		targetingForwardSpeed *= swerve.getMaximumVelocity();
		targetingForwardSpeed *= -1.0;

		// this is bad but we will fix later
		targetingForwardSpeed -= 4.3;
		//double out = targetingForwardSpeed > 0 ? targetingForwardSpeed : 0;
		return targetingForwardSpeed;
	}

	public void drive(boolean fieldRelative) {
		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		double xSpeed = MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * swerve.getMaximumVelocity();

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		double ySpeed = MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * swerve.getMaximumVelocity();

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		double rot = MathUtil.applyDeadband(driverController.getRightX(), 0.02) * swerve.getMaximumAngularVelocity();

		// while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
		if(driverController.b().getAsBoolean())
		{
			final double rot_limelight = limelight_aim_proportional();
			rot = rot_limelight;

			final double forward_limelight = limelight_range_proportional();
			xSpeed = forward_limelight;

			//while using Limelight, turn off field-relative driving.
			fieldRelative = false;
		}

		swerve.drive(xSpeed, ySpeed, rot, fieldRelative, 0.002);
		}

}