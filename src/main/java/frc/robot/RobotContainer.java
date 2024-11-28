// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IndexNote;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.misc.LED;
import frc.robot.subsystems.misc.ProximitySensor;
import frc.robot.subsystems.shooter.ShooterSubsystem;
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
	private static State ROBOT_STATE = State.IDLE;
	
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
		
		CommandScheduler.getInstance()
        	.onCommandInitialize(
            	command -> Logger.recordOutput("Current Command", command.getName()));
		
		this.configureDefaultCommands();
		this.configureBindings();
		this.registerPathplannerCommands();

		this.autoChooser = new LoggedDashboardChooser<Command>("auto/Auto Chooser", AutoBuilder.buildAutoChooser());

		// Sets the color of the LEDS once
		//this.led.setColor(255, 255, 255);
	}

	private void registerPathplannerCommands() {
	}

	private void configureDefaultCommands() {
		this.swerve.setDefaultCommand(this.swerve.driveCommand(driverController));

		/*this.shooter.setDefaultCommand(
			Commands.either(
				this.shooter.setVelocityFromDistance(this.swerve::getDistanceFromSpeaker),
				this.shooter.stop(),
				() -> sensor.isNoteIndexed() && this.swerve.getDistanceFromSpeaker() < 4
			)
		);*/
		this.shooter.setDefaultCommand(this.shooter.stop());
		this.feeder.setDefaultCommand(this.feeder.stop());

		this.led.setDefaultCommand(
			this.led.setColor(() -> new Color8Bit(255, 255, 255))
		);
	}

	private void configureBindings() {
		// Press A to Zero Gyro
		driverController.a().onTrue((Commands.runOnce(swerve::zeroGyro)));

		// right bumper
		driverController.rightBumper().whileTrue(
			new AutoShoot()
		);

		// B -> Drive to Amp
		driverController.b().whileTrue(
			this.swerve.driveToAmp()
		);

		// right bumper -> shoot
		new JoystickButton(operatorController, 6)
			.whileTrue(
				Commands.parallel(
					swerve.turnToSpeaker(),
					shooter.shoot()
				)
			);		

		new JoystickButton(operatorController, 6)
		.whileTrue(
			Commands.parallel(
				swerve.turnToSpeaker(),
				shooter.setVelocityFromDistance(swerve::getDistanceFromSpeaker)
			)
		);	

		// left bumper -> intake
		new JoystickButton(operatorController, 5)
		.whileTrue(
			Commands.repeatingSequence(feeder.intake())
		);	
	}

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	/**
	 * updates pose estimation using data from a Limelight camera and adjusts the
	 * robot's orientation based on the vision measurements.
	 */
	public void updatePoseEstimation() {
		swerve.getSwerveDrive().updateOdometry();

		if (RobotBase.isSimulation()) {
			return;
		}

		LimelightHelpers.SetRobotOrientation("limelight-shooter", swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
		LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-shooter");
		
		try {
			/* if our angular velocity is greater than 720 degrees per second, ignore vision updates */ 
			if (Math.abs(swerve.getAngularVelocity()) > 720) return;
			if(mt2.tagCount == 0) return;
			
			double estimatedRotation = mt2.pose.getRotation().getRadians();
			Rotation3d newRotation = new Rotation3d(0, 0, estimatedRotation);
			swerve.getSwerveDrive().setGyro(newRotation);
			swerve.getSwerveDrive().addVisionMeasurement(mt2.pose, mt2.timestampSeconds, Constants.VISION.VISION_MEASUREMENT_STD_DEV);
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
		if (ROBOT_STATE == newState) {
			return;
		}

        ROBOT_STATE = newState;
        SmartDashboard.putString("robot/robot state", ROBOT_STATE.toString());
    }
}