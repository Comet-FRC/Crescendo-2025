// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import java.util.logging.Level;

import com.pathplanner.lib.auto.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

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
	// TODO: measure limelight values
	private final VisionSubsystem limelightShooter = new VisionSubsystem("limelight-shooter", 0, 0);
	private final VisionSubsystem limelightIntake = new VisionSubsystem("limelight-shooter", 0, 0);

	private final LaserCan laserCan = new LaserCan(Constants.Feeder.laserCanID);

	/* Booleans */
	private boolean hasNote = false;
	
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

		// TODO: Maybe move this stuff to a dedicated sensor subsystem
		try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            Robot.getLogger().log(Level.SEVERE, "test");
        }
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

	public SwerveSubsystem getSwerveSubsystem() {
		return swerve;
	}

	public void updateOdometry() {
		SwerveDrivePoseEstimator poseEstimator = swerve.getPoseEstimator();

		poseEstimator.update(
			swerve.getHeading(), 
			swerve.getModulePositions());
		
		boolean doRejectUpdate = false;
		
		LimelightHelpers.SetRobotOrientation("limelight-shooter", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");
		if(Math.abs(swerve.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
		{
		doRejectUpdate = true;
		}
		if(mt2.tagCount == 0)
		{
		doRejectUpdate = true;
		}
		if(!doRejectUpdate)
		{
		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
		poseEstimator.addVisionMeasurement(
			mt2.pose,
			mt2.timestampSeconds);
		}
  	}

	/**
	 * Makes the robot move. Contains code to control auto-aim button presses
	 * @param fieldRelative whether or not the robot is driving in field oriented mode
	 */
	public void drive(boolean fieldRelative) {
		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		double xSpeed = MathUtil.applyDeadband(driverController.getLeftY(), 0.02) * swerve.getMaximumVelocity();

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		double ySpeed = MathUtil.applyDeadband(driverController.getLeftX(), 0.02) * swerve.getMaximumVelocity();

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		double rot = MathUtil.applyDeadband(driverController.getRightX(), 0.02) * swerve.getMaximumAngularVelocity();

		// while the B-button is pressed, overwrite some of the driving values with the output of our limelight methods
		if(driverController.b().getAsBoolean())
		{
			final double rot_limelight = limelightShooter.aim_proportional();
			rot = rot_limelight;

			final double forward_limelight = limelightShooter.range_proportional(100);
			xSpeed = forward_limelight;

			//while using Limelight, turn off field-relative driving.
			fieldRelative = false;
		}

		// while the Y-button is pressed, overwrite some of the driving values with the output of our limelight methods
		else if(driverController.y().getAsBoolean()) // add hasNote = false && when we get the thing
		{
			final double rot_limelight = limelightIntake.aim_proportional();
			rot = rot_limelight;

			// Move forward at 1 m/s
			xSpeed = 1;

			//while using Limelight, turn off field-relative driving.
			fieldRelative = false;
		}

		// TODO: This was 0.002 but should be 0.02. Check if this changes anything though
		swerve.drive(xSpeed, ySpeed, rot, fieldRelative, 0.02);
	}

	public void updateNoteStatus() {
		LaserCan.Measurement measurement = laserCan.getMeasurement();

		if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
			//System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");

		hasNote = measurement.distance_mm <= 50;
	}

}