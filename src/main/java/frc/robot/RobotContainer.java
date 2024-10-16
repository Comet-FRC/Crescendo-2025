// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PrepShootCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem.Speed;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.LimelightIntake;
import frc.robot.subsystems.Vision.LimelightShooter;

import java.io.File;
import java.util.logging.Level;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	/* Controllers */
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final Joystick operatorController = new Joystick(1);

	/* Subsystems */
	private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
	private final IntakeSubsystem intake = new IntakeSubsystem();
	private final FeederSubsystem feeder = new FeederSubsystem();
	private final ShooterSubsystem shooter = new ShooterSubsystem();

	public final LimelightShooter limelightShooter = new LimelightShooter("limelight-shooter");
	public final LimelightIntake limelightIntake = new LimelightIntake("limelight-intake");

	private final LaserCan laserCan = new LaserCan(Constants.Feeder.laserCanID);

	/* Robot states */
	public boolean hasNote = false;
	private State robotState = State.IDLE;

	private final Field2d field = new Field2d();

	private boolean isForwardOverriden = false;
	private boolean isStrafeOverriden = false;
	private boolean isRotationOverriden = false;
	private double forwardSpeedOverride = 0;
	private double strafeSpeedOverride = 0;
	private double rotationalSpeedOverride = 0;
	
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		DriverStation.silenceJoystickConnectionWarning(true);
		try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            Robot.getLogger().log(Level.SEVERE, "LaserCAN configuration failed!");
        }

		// Press A to Zero Gyro
		driverController.a().onTrue((Commands.runOnce(swerve::zeroGyro)));

		driverController.rightBumper()
			.and(driverController.leftBumper().negate())
			.onTrue(
				new PrepShootCommand(shooter, limelightShooter)
				.andThen(new ShootCommand(shooter, feeder))
			);

		driverController.leftBumper()
			.and(driverController.rightBumper().negate())
			.onTrue(new IntakeCommand(intake, feeder, limelightIntake, laserCan));

		driverController.back()
			.onTrue(new OuttakeCommand(shooter, feeder, intake));
		
		SmartDashboard.putNumber("robot/desired distance", Constants.SHOOT_DISTANCE);
		SmartDashboard.putData("robot/field", field);
	}

	public SwerveSubsystem getSwerveSubsystem() {
		return swerve;
	}

	public void updateVision() {

		limelightIntake.updateVisionData();
		limelightShooter.updateVisionData();

		swerve.getSwerveDrive().updateOdometry();

		boolean doRejectUpdate = false;

		LimelightHelpers.SetRobotOrientation("limelight-shooter", swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
		
		// if our angular velocity is greater than 720 degrees per second, ignore vision updates
		if (Math.abs(swerve.getRate()) > 720) {
			doRejectUpdate = true;
		} else if(mt2.tagCount == 0) {
			doRejectUpdate = true;
		}
		
		if(!doRejectUpdate) {
			//double estimatedRotation = -mt2.pose.getRotation().getDegrees();
			//swerve.getSwerveDrive().setGyro(new Rotation3d(estimatedRotation, 0, 0));
			swerve.getSwerveDrive().addVisionMeasurement(mt2.pose, mt2.timestampSeconds, Constants.VISION_MEASUREMENT_STD_DEV);
		}

		SmartDashboard.putNumber("robot/estimated rotation", mt2.pose.getRotation().getDegrees());
		field.setRobotPose(swerve.getSwerveDrive().getPose());
  	}

	public enum State {
		IDLE,
		INTAKING,
		OUTTAKING,
		PREPPING,
		SHOOTING
	}

	public void setRobotState(State state) {
		// Already on the same state
		if (robotState == state) {
			return;
		}

		robotState = state;

		SmartDashboard.putString("robot/robot state", robotState.toString());
	}

	/**
	 * Makes the robot move. Contains code to control auto-aim button presses
	 * @param fieldRelative whether or not the robot is driving in field oriented mode
	 */
	public void drive(boolean fieldRelative) {


		double forwardSpeed = -MathUtil.applyDeadband(driverController.getLeftY(), 0.02) * swerve.getMaximumVelocity();
		double strafeSpeed = -MathUtil.applyDeadband(driverController.getLeftX(), 0.02) * swerve.getMaximumVelocity();
		double rotationalSpeed = -MathUtil.applyDeadband(driverController.getRightX(), 0.02) * swerve.getMaximumAngularVelocity();


		if (isForwardOverriden) {
			forwardSpeed = forwardSpeedOverride;
		}

		if (isStrafeOverriden) {
			strafeSpeed = strafeSpeedOverride;
		}

		if (isRotationOverriden) {
			rotationalSpeed = rotationalSpeedOverride;
		}

		switch (robotState) {
			case OUTTAKING:
				break;
			case INTAKING:
				fieldRelative = false;
				break;
			case PREPPING:
				if (!limelightShooter.hasTarget()) break;
				fieldRelative = false;
				break;
			case SHOOTING:
				fieldRelative = false;
				break;
			default:
				robotState = State.IDLE;
				fieldRelative = true;
				shooter.stop();
				intake.stop();
				feeder.stop();
				break;
		}

		swerve.drive(forwardSpeed, strafeSpeed, rotationalSpeed, fieldRelative, 0.02);
		
		/*
		 * The loop is over so we reset variables.
		 * Next thing that runs is the command scheduler
		 */
		isForwardOverriden = false;
		isStrafeOverriden = false;
		isRotationOverriden = false;
	}

	/**
	 * Sets hasNote
	 */
	public void setNoteStatus(boolean hasNote) {
		this.hasNote = hasNote;
		//SmartDashboard.putNumber("robot/proximityDistance", measurement.distance_mm);
		SmartDashboard.putBoolean("robot/hasNote", hasNote);
	}

	/**
	 * Sets {@link #hasNote} to true if the LaserCAN detects an object, false otherwise
	 * @return returns the status of the note;
	 */
	public void updateNoteStatus() {
		LaserCan.Measurement measurement = laserCan.getMeasurement();

		if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
			return;

		hasNote = measurement.distance_mm <= 75;
		//SmartDashboard.putNumber("robot/proximityDistance", measurement.distance_mm);
		SmartDashboard.putBoolean("robot/hasNote", hasNote);
	}

	public void setForwardSpeedOverride(double forwardSpeed) {
		if (!isForwardOverriden)
			isForwardOverriden = true;
		forwardSpeedOverride = forwardSpeed;
	}

	public void setStrafeSpeedOverride(double strafeSpeed) {
		if (!isStrafeOverriden)
			isStrafeOverriden = true;
		strafeSpeedOverride = strafeSpeed;
	}

	public void setRotationalSpeedOverride(double rotationalSpeed) {
		if (!isRotationOverriden)
			isRotationOverriden = true;
		rotationalSpeedOverride = rotationalSpeed;
	}
}