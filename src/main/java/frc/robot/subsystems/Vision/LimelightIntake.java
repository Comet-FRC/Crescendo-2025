package frc.robot.subsystems.Vision;

import frc.robot.Robot;

public class LimelightIntake extends Limelight {
    public LimelightIntake(String name) {
        super(name);
    }

    public double strafe_proportional(double kP) {
        kP *= Math.abs(getTY());
        kP /= 100.0;
        
		double targetingSidewaysSpeed = getTX() * kP;
		targetingSidewaysSpeed *= Robot.getInstance().getRobotContainer().getSwerveSubsystem().getMaximumVelocity();
		targetingSidewaysSpeed *= -1;
		return targetingSidewaysSpeed;
    }

    @Override
    protected double getRawTX() {
        final double a = 0.01024;
		final double b = 0.1129;
		final double c = 13.9;

        double raw_ty = getRawTY();

		double target_tx =  a*raw_ty*raw_ty + b*raw_ty + c;
        
		double delta_tx = target_tx - LimelightHelpers.getTX(getName());

        return delta_tx;
    }

    @Override
    protected double getRawTY() {
        return LimelightHelpers.getTY(getName());
    }
}
