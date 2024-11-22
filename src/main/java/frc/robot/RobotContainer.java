// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.misc.LED;
import frc.robot.subsystems.misc.ProximitySensor;
import frc.robot.utils.LimelightHelpers;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	/* Subsystems */
	private final SwerveSubsystem swerve;
	private final IntakeSubsystem intake;
	private final FeederSubsystem feeder;
	private final ShooterSubsystem shooter;

	private final ProximitySensor sensor;
	private final LED led;

	private final CommandXboxController driverController;
	private final Joystick operatorController;
	
	private final LoggedDashboardChooser<Command> autoChooser;

	/* Robot states */
	private static State robotState = State.IDLE;
	
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		this.swerve = SwerveSubsystem.getInstance();
		this.intake = IntakeSubsystem.getInstance();
		this.feeder = FeederSubsystem.getInstance();
		this.shooter = ShooterSubsystem.getInstance();

		this.sensor = ProximitySensor.getInstance();
		this.led = LED.getInstance();

		this.driverController = new CommandXboxController(0);
		this.operatorController = new Joystick(1);
		DriverStation.silenceJoystickConnectionWarning(true);
		
		this.registerPathplannerCommands();
		this.configureBindings();

		this.autoChooser = new LoggedDashboardChooser<Command>("auto/Auto Chooser", AutoBuilder.buildAutoChooser());

		this.configureDefaultCommands();

		// Sets the color of the LEDS once
		this.led.setColor(255, 255, 255);
	}

	private void registerPathplannerCommands() {
	}

	private void configureBindings() {
		// Press A to Zero Gyro
		driverController.a().onTrue((Commands.runOnce(swerve::zeroGyro)));

		// right bumper -> prep, then shoot
		driverController.rightBumper().whileTrue(
			new AutoShoot()
		);

		// B -> Drive to Amp
		driverController.b().whileTrue(
			this.swerve.driveToAmp()
		);

		// back button (not B button)
		new JoystickButton(operatorController, 7).whileTrue(
			new OuttakeCommand()
		);

		// start button -> intake
		new JoystickButton(operatorController, 8).whileTrue(
			new IntakeCommand()
		);

		// right bumper -> spin shooter wheels

		new JoystickButton(operatorController, 6).whileTrue(
			new ShootCommand(shooter)
			.finallyDo(
				() -> {
					Logger.recordOutput("shooter/last shoot distance", swerve.getDistanceFromSpeaker());
					Logger.recordOutput("shooter/last shoot pose", swerve.getPose().toString());
				}
			)
		);		

		// left bumper -> intake note, then index it
		new JoystickButton(operatorController, 5).whileTrue(
			new SequentialCommandGroup(
				Commands.runOnce(() -> setState(State.INTAKING)),
				new IntakeCommand()
					.onlyWhile(() -> !sensor.hasObject())
					.onlyWhile(() -> feeder.getTorqueCurrent() > -25),
					Commands.runOnce(() -> setState(State.REVVING))
			)
		);
	}

	private void configureDefaultCommands() {
		swerve.setDefaultCommand(
			swerve.driveCommand(
				() -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.02),
				() -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.02),
				() -> -MathUtil.applyDeadband(driverController.getRightX(), 0.02)
			)
		);
	}

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	public void updateVision() {
		swerve.getSwerveDrive().updateOdometry();

		if (RobotBase.isSimulation()) {
			return;
		}

		boolean doRejectUpdate = false;

		LimelightHelpers.SetRobotOrientation("limelight-shooter", swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
		
		try {
			// if our angular velocity is greater than 720 degrees per second, ignore vision updates
			if (Math.abs(swerve.getRate()) > 720) {
				doRejectUpdate = true;
			} else if(mt2.tagCount == 0) {
				doRejectUpdate = true;
			}
			
			if(!doRejectUpdate) {
				//double estimatedRotation = mt2.pose.getRotation().getDegrees();
				//swerve.getSwerveDrive().setGyro(new Rotation3d(estimatedRotation, 0, 0));
				swerve.getSwerveDrive().addVisionMeasurement(mt2.pose, mt2.timestampSeconds, Constants.VISION.VISION_MEASUREMENT_STD_DEV);
			}

			SmartDashboard.putNumber("robot/estimated rotation", mt2.pose.getRotation().getDegrees());
		} catch (Exception e) {
		}
  	}

	public enum State {
		IDLE,
		INTAKING,
		OUTTAKING,
		REVVING,
		SHOOTING
	}

	public static void setState(State newState) {

        // Already on the same state
		if (robotState == newState) {
			return;
		}

        robotState = newState;
        SmartDashboard.putString("robot/robot state", robotState.toString());
    }
}