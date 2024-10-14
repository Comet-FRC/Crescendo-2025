package frc.robot.subsystems.Vision;

import frc.robot.Robot;

public abstract class Limelight {

    private final String name;

    public Limelight(String name) {
        this.name = name;
    }

    public double proportionalYaw(double kP) {
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
		// your limelight 3 feed, tx should return roughly 31 degrees.
		double targetingAngularVelocity = tx() * kP;

		// convert to radians per second for our drive method
		targetingAngularVelocity *= Robot.getInstance().getRobotContainer().getSwerveSubsystem().getMaximumAngularVelocity();

		//invert since tx is positive when the target is to the right of the crosshair
		targetingAngularVelocity *= -1.0;

		return targetingAngularVelocity;
    };

    public boolean hasTarget() {
        return LimelightHelpers.getTV(name);
    }

    public String getName() {
        return name;
    }

    public abstract double ty();
    public abstract double tx();
    
}
