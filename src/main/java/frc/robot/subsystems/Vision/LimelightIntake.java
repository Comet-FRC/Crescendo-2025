package frc.robot.subsystems.Vision;

import frc.robot.Robot;

public class LimelightIntake extends Limelight {
    public LimelightIntake(String name) {
        super(name);
    }

    public double proportionalY(double kP) {
        double delta_tx = tx();
		double targetingSidewaysSpeed = delta_tx * kP;
		targetingSidewaysSpeed *= Robot.getInstance().getRobotContainer().getSwerveSubsystem().getMaximumVelocity();
		targetingSidewaysSpeed *= -1;
		return targetingSidewaysSpeed;
    }

    @Override
    public double ty() {
        return LimelightHelpers.getTY(getName());
    }

    @Override
    public double tx() {
        final double a = 0.0345339;
		final double n = 1.72763;
		final double b = 14.174;
		double target_tx =  a * Math.pow(ty(), n) + b;
		double delta_tx = target_tx - LimelightHelpers.getTX(getName());

        return delta_tx;
    }
}
