// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.PrepareShootCommand;
import frc.robot.commands.AlignToSpeakerCommand;
import frc.robot.commands.EjectCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Speed;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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

	private final Trigger alignButton = new JoystickButton(operatorController, 4);
	private final Trigger intakeButton = new JoystickButton(operatorController, 5);
	private final Trigger shooterButton = new JoystickButton(operatorController, 6);
	private final Trigger ejectButton = new JoystickButton(operatorController, 7);

	/* Subsystems */
	private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
	private final IntakeSubsystem intake = new IntakeSubsystem();
	private final FeederSubsystem feeder = new FeederSubsystem();
	private final ShooterSubsystem shooter = new ShooterSubsystem();
	private final VisionSubsystem vision = new VisionSubsystem(drivebase, "limelight");

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);


		/* REGISTERING COMMANDS FOR PATHPLANNER */
		configureAutonCommands();

		/* Configure the trigger bindings */
		configureBindings();

		/*
		Applies deadbands and inverts controls because joysticks are back-right positive
		while robot controls are front-left positive left stick controls translation
		right stick controls the rotational velocity buttons are quick rotation
		positions to different ways to face
		WARNING: default buttons are on the same buttons as the ones defined in configureBindings
		*/
		AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
			() -> -MathUtil.applyDeadband(driverController.getLeftY(),
				OperatorConstants.LEFT_Y_DEADBAND),
			() -> -MathUtil.applyDeadband(driverController.getLeftX(),
				OperatorConstants.LEFT_X_DEADBAND),
			() -> -MathUtil.applyDeadband(driverController.getRightX(),
				OperatorConstants.RIGHT_X_DEADBAND),
			driverController.getHID()::getYButtonPressed,
			driverController.getHID()::getAButtonPressed,
			driverController.getHID()::getXButtonPressed,
			driverController.getHID()::getBButtonPressed);

		/*
		Applies deadbands and inverts controls because joysticks are back-right positive
		while robot controls are front-left positive left stick controls translation
		right stick controls the desired angle NOT angular rotation
		*/
		Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
			() -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
			() -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
			() -> driverController.getRightX(),
			() -> driverController.getRightY());

		/*
		Applies deadbands and inverts controls because joysticks are back-right positive
		while robot controls are front-left positive left stick controls translation
		right stick controls the angular velocity of the robot
		*/
		Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
			() -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
			() -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
			() -> driverController.getRightX() * 0.5);

		Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
			() -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
			() -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
			() -> driverController.getRawAxis(2));

		drivebase.setDefaultCommand(
			!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

		autoChooser = AutoBuilder.buildAutoChooser();
			SmartDashboard.putData("auto/Auto Chooser", autoChooser);
	}

	private void configureAutonCommands() {
		/* Subwoofer Shot */
		NamedCommands.registerCommand("Fixed SW shot",
			Commands.runOnce(() -> {SmartDashboard.putString("Auto Status", "Begin SW shot");})
				.andThen(
					(new ShootCommand(shooter, feeder, Speed.SUBWOOFER)
					.raceWith(Commands.waitSeconds(1.50))))
				.andThen(Commands.runOnce(() -> {  SmartDashboard.putString("Auto Status", "SW complete"); }))
			);
		
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
		zeroGyroButton.onTrue((Commands.runOnce(drivebase::zeroGyro)));
		driverController.b().whileTrue(
			Commands.deferredProxy(() -> drivebase.driveToPose(
				new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
			));

		
		alignButton.whileTrue(new AlignToSpeakerCommand(drivebase, vision).withName("Align to Speaker"));

		ejectButton.whileTrue(new EjectCommand(intake, feeder).withName("Eject"));
		
		shooterButton.whileTrue(
			new PrepareShootCommand(shooter, feeder, Speed.SPEAKER)
			.andThen(new ShootCommand(shooter, feeder, Speed.SPEAKER)
			.withName("Shoot Command")));
		
		intakeButton.whileTrue(new IntakeCommand(intake, feeder).withName("Intake"));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return new PathPlannerAuto("New Auto");
	}

	public void setDriveMode() {
		//drivebase.setDefaultCommand();
	}

	public void setMotorBrake(boolean brake) {
		drivebase.setMotorBrake(brake);
	    }
}