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
        final double a = 0.002831;
		final double b = 0.4219;
		final double c = 11.37;

        double raw_ty = getRawTY();

		double target_tx =  a*raw_ty*raw_ty + b*raw_ty + c;
        
		double delta_tx = target_tx - LimelightHelpers.getTX(getName());

        return delta_tx;
    }

    @Override
    protected double getRawTY() {
        return LimelightHelpers.getTY(getName());
    }

    public enum LED_MODE {
        OFF,
        ON,
        BLINKING
    }

    public void setLEDMode(LED_MODE mode) {
        switch (mode) {
            case OFF:
                LimelightHelpers.setLEDMode_ForceOff(getName());
                break;
            case ON:
                LimelightHelpers.setLEDMode_ForceOn(getName());
                break;
            case BLINKING:
                LimelightHelpers.setLEDMode_ForceBlink(getName());
                break;
        }
    }
}
