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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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
	private final Trigger intakeButton = new JoystickButton(driverController.getHID(), XboxController.Button.kLeftBumper.value);
	private final Trigger shooterButton = new JoystickButton(operatorController, 6);
	private final Trigger ejectButton = new JoystickButton(operatorController, 7);
	/* Subsystems */
	private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
	private final IntakeSubsystem intake = new IntakeSubsystem();
	private final FeederSubsystem feeder = new FeederSubsystem();
	private final ShooterSubsystem shooter = new ShooterSubsystem();

	private final VisionSubsystem limelightShooter = new VisionSubsystem("limelight-shooter", 40, 13);
	private final VisionSubsystem limelightIntake = new VisionSubsystem("limelight-intake", -10, 16.5);

	private final LaserCan laserCan = new LaserCan(Constants.Feeder.laserCanID);

	/* Robot states */
	public boolean hasNote = false;
	private State robotState = State.IDLE;
	private Timer shootTimer = new Timer();
	
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
		configureAutonCommands(	);
		/* Configure the trigger bindings */
		configureBindings();

		autoChooser = AutoBuilder.buildAutoChooser();
			SmartDashboard.putData("auto/Auto Chooser", autoChooser);

		// TODO: Maybe move this stuff to a dedicated sensor subsystem
		try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            Robot.getLogger().log(Level.SEVERE, "test");
        }
	}

	private void configureAutonCommands() {
		/* Shoot */
		NamedCommands.registerCommand("SubWoof",
			Commands.runOnce(() -> {SmartDashboard.putString("Auto Status", "Begin SW shot");})
				.andThen(new AutonPrepareShootCommand(shooter, Speed.SPEAKER))
				.andThen(new AutonShootCommand(shooter, feeder, intake, Speed.SPEAKER))

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
		zeroGyroButton.onTrue((Commands.runOnce(swerve::zeroGyro)));

		ampButton.whileTrue(new ShootCommand(shooter, Speed.AMP));

		ejectButton.whileTrue(new EjectCommand(intake, feeder).withName("Eject"));

		shooterButton.whileTrue(
			new ShootCommand(shooter, Speed.SPEAKER)
			.withName("Shoot Command"));



		//intakeButton.whileTrue(new IntakeCommand(intake, feeder).withName("Intake"));
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
		
		// if our angular velocity is greater than 720 degrees per second, ignore vision updates
		if (Math.abs(swerve.getRate()) > 720) {
			doRejectUpdate = true;
		} if(mt2.tagCount == 0) {
			doRejectUpdate = true;
		} if(!doRejectUpdate) {
			poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
			poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
		}
  	}

	public enum State {
		IDLE,
		INTAKING,
		PREPPING,
		SHOOTING
	}

	/**
	 * Updates the robot state
	 */
	public void updateState() {
		if (driverController.b().getAsBoolean() && hasNote) {
			if (robotState == State.PREPPING || robotState == State.SHOOTING)
				return;
			robotState = State.PREPPING;

		} else if (driverController.y().getAsBoolean() && !hasNote) {
			robotState = State.INTAKING;

		} else {
			robotState = State.IDLE;
		}

		SmartDashboard.putString("robot/robot state", robotState.toString());
	}

	/**
	 * Makes the robot move. Contains code to control auto-aim button presses
	 * @param fieldRelative whether or not the robot is driving in field oriented mode
	 */
	public void drive(boolean fieldRelative) {
		double xSpeed = MathUtil.applyDeadband(driverController.getLeftY(), 0.02) * swerve.getMaximumVelocity();
		double ySpeed = MathUtil.applyDeadband(driverController.getLeftX(), 0.02) * swerve.getMaximumVelocity();
		double rotationalSpeed = -MathUtil.applyDeadband(driverController.getRightX(), 0.02) * swerve.getMaximumAngularVelocity();

		switch (robotState) {
			case INTAKING:
				fieldRelative = false;

				// Move forward at 0.75 m/s
				xSpeed = -0.75;

				if (limelightIntake.hasTarget()) {
					rotationalSpeed = limelightIntake.aim_proportional(0.035);
					if (rotationalSpeed > 0.01)
						xSpeed = 0;
				}

				intake.intake();
				feeder.intake();

				break;
			case PREPPING:
				if (limelightShooter.hasTarget()) {
					rotationalSpeed = limelightShooter.aim_proportional(0.035);
					xSpeed = limelightShooter.range_proportional(70, 57.5);
					fieldRelative = false;
				}
				//SmartDashboard.putNumber("robot/rotationalSpeed", rotationalSpeed);
				if (Math.abs(rotationalSpeed) < 0.02 && Math.abs(xSpeed) < 0.02 && shooter.isReady(false)) {
					robotState = State.SHOOTING;
					break;
				}
				shooter.shoot(Speed.SPEAKER);
				break;
			case SHOOTING:
				fieldRelative = false;
				shootTimer.restart();
				feeder.intake();
				if (shootTimer.hasElapsed(Constants.Shooter.postShotTimeout)) {
					robotState = State.IDLE;
					feeder.stop();
				}
				break;
			default:
				robotState = State.IDLE;
				fieldRelative = true;
				shooter.stop();
				intake.stop();
				feeder.stop();
				break;
		}

		swerve.drive(xSpeed, ySpeed, rotationalSpeed, fieldRelative, 0.02);
	}

	/**
	 * Sets {@link #hasNote} to true if the LaserCAN detects an object, false otherwise
	 */
	public void updateNoteStatus() {
		LaserCan.Measurement measurement = laserCan.getMeasurement();

		if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
			return;

		//System.out.println(measurement.distance_mm);
		hasNote = measurement.distance_mm <= 75;
		SmartDashboard.putNumber("robot/proximityDistance", measurement.distance_mm);
		SmartDashboard.putBoolean("robot/hasNote", hasNote);
	}

}