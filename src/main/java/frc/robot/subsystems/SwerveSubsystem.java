// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.utils.AllianceColor;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    
    /* Singleton */

    private static SwerveSubsystem instance = null;

    public static SwerveSubsystem getInstance() {
        if (instance == null) instance = new SwerveSubsystem();

        return instance;
    }

    /* Implementation */

    private final SwerveDrive swerveDrive;
    private final ProfiledPIDController rotationPID;
    private final PathConstraints autonConstraints;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    private SwerveSubsystem() {

        File directory = new File(Filesystem.getDeployDirectory(), "swerve/neo");

        try {
            this.swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.SWERVE.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        // This adds the trajectories on the field display
        // Subscribe to PathPlanner path updates
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            this.swerveDrive.field.getObject("path").setPoses(poses);
        });

        // Subscribe to PathPlanner target pose updates
        /*
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            swerveDrive.field.getObject("target").setPose(pose);
        });
        */

        // Heading correction should only be used while controlling the robot via angle.
        //swerveDrive.setHeadingCorrection(true);
        //swerveDrive.setAutoCenteringModules(true);
        if (RobotBase.isSimulation()) {
			this.swerveDrive.setHeadingCorrection(true);
		}
        this.swerveDrive.setCosineCompensator(false);
        this.setupPathPlanner();
        
        /*
         * DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER
         * see https://pathplanner.dev/pplib-pathfinding.html#java-warmup
         */
        PathfindingCommand.warmupCommand().schedule();

        this.rotationPID = new ProfiledPIDController(
            0.06,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(360.0, 360.0 * 2.0)
        );
        this.rotationPID.enableContinuousInput(-180, 180);
        this.rotationPID.setTolerance(0.1, 0.4);
        this.autonConstraints = new PathConstraints(
            this.getMaximumVelocity(), 4.0,
            this.getMaximumAngularVelocity(),
            Units.degreesToRadians(720)
        );
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    private void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                this::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        Constants.AUTON.TRANSLATION_PID,
                        Constants.AUTON.ANGLE_PID,
                        this.getMaximumVelocity(),
                        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        new ReplanningConfig()
                ),
                () -> AllianceColor.isRed(),
                this
        );
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
                pose,
                this.autonConstraints,
                0.0,
                0.0
        );
    }

    public Command driveToAmp() {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("AmpAlign"),
            new PathConstraints(
                this.getMaximumVelocity(), 4.0,
                this.getMaximumAngularVelocity(),
                Units.degreesToRadians(720)
            )
        );
    }
    
    public Translation2d getSpeakerPosition() {
        return AllianceColor.isRed()  ? new Translation2d(16.23, 5.55) : new Translation2d(0.3, 5.55);
    }

    public Translation2d getAmpPosition() {
        return AllianceColor.isRed() ? new Translation2d(14.701, 8.204) : new Translation2d(1.841, 8.204);
    }

    public double getDistance(Translation2d targetPose) {
        return this.getPose().getTranslation().getDistance(targetPose);
    }

    public double getDistance(Pose2d targetPose) {
        return this.getPose().getTranslation().getDistance(targetPose.getTranslation());
    }

    public double getDistanceFromSpeaker() {
        return this.getDistance(this.getSpeakerPosition());
    }

    public double getDistanceFromAmp() {
        return this.getDistance(this.getAmpPosition());
    }

    /**
     * @see https://github.com/Team-8-bit/2024-Sonic/blob/develop/robot/src/main/kotlin/org/team9432/robot/RobotPosition.kt
     * @param targetPosition
     * @return the angle between the robots current pose and the target position
     */
    public Rotation2d getAngleTo(Translation2d targetPosition) {
        return new Rotation2d(
            Math.atan2(
                targetPosition.getY() - this.getPose().getY(),
                targetPosition.getX() - this.getPose().getX()
            )
        );
    }

    

    public Rotation2d getAngleToSpeaker() {
        return getAngleTo(getSpeakerPosition());
    }

    private Command turnToAngle(Supplier<Rotation2d> angleSupplier) {
        return new FunctionalCommand(
            () -> {
                rotationPID.setGoal(angleSupplier.get().getDegrees());
                rotationPID.reset(this.getPose().getRotation().getDegrees(), this.getAngularVelocity());
            },
            () -> {
                double angularVelocity = rotationPID.calculate(this.getPose().getRotation().getDegrees());
                ChassisSpeeds speed = ChassisSpeeds.fromFieldRelativeSpeeds(0,0, angularVelocity, this.getYaw());
                this.drive(speed);
            }, 
            interrupted -> {},
            rotationPID::atSetpoint,
            this
        );
    }

    public Command turnToSpeaker() {
        return this.turnToAngle(this::getAngleToSpeaker);
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, swerveDrive, 12),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, swerveDrive),
                3.0, 5.0, 3.0);
    }

    public Command driveCommand(CommandXboxController controller) {
        return this.driveCommand(
            () -> -MathUtil.applyDeadband(controller.getLeftY(), 0.02),
            () -> -MathUtil.applyDeadband(controller.getLeftX(), 0.02),
            () -> -MathUtil.applyDeadband(controller.getRightX(), 0.02)
        );
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction.
     * @param translationY     Translation in the Y direction.
     * @param angularRotationX Angular velocity of the robot to set.
     * @return Drive command.
     */
    public Command driveCommand(
        DoubleSupplier translationX,
        DoubleSupplier translationY,
        DoubleSupplier angularRotationX
    ) {
        return run(() -> {
            double xVelocity = Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity();
            double yVelocity = Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity();
            double angularVelocity = Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity();

            swerveDrive.drive(
                new Translation2d(xVelocity, yVelocity),
                angularVelocity,
                true,
                false);
        });
    }

    /**
     * Secondary method for controlling the drivebase. Given a simple {@link ChassisSpeeds} set the
     * swerve module states, to achieve the goal.
     *
     * @param velocity The desired robot-oriented {@link ChassisSpeeds} for the robot to achieve.
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is
     *         available.
     */
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing
     * forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                SWERVE.MAX_SPEED);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                SWERVE.MAX_SPEED);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public Rotation2d getYaw() {
        return swerveDrive.getYaw();
    }

    public void stop() {
        swerveDrive.drive(new ChassisSpeeds(0,0,0));
    }

    public double getMaximumAngularVelocity() {
        return swerveDrive.getMaximumAngularVelocity();
    }

    public double getMaximumVelocity() {
        return swerveDrive.getMaximumVelocity();
    }

    public SwerveModulePosition[] getModulePositions() {
        return swerveDrive.getModulePositions();
    }
    
    /**
     * returns the angular velocity by retrieving the rate from a Pigeon2 gyro sensor in a swerve drive system.
     * 
     * @return the angular velocity of the swerve drive, in meters per second
     */
    public double getAngularVelocity() {
        return ((Pigeon2)(swerveDrive.getGyro().getIMU())).getRate();
    }

    /*public Command resetOdometryToEstimated(){
        return Commands.runOnce(() -> swerveDrive.setGyro(swerveDrive.););
    }*/

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("robot/shooter/distance from speaker", this.getDistanceFromSpeaker());
    }
}