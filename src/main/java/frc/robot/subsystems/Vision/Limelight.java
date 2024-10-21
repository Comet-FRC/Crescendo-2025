package frc.robot.subsystems.Vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public abstract class Limelight {
    private final ArrayList<LimelightData> limelightData;
    private final String name;

    private double tx;
    private double ty;
    private boolean hasTarget;

    class LimelightData {
        private double tx;
        private double ty;
        private boolean isValid;
        
        public LimelightData(double tx, double ty, boolean isValid) {
            this.tx = tx;
            this.ty = ty;
            this.isValid = isValid;
        }

        public double getTX() {
            return tx;
        }

        public double getTY() {
            return ty;
        }

        public boolean isValid() {
            return isValid;
        }
    }

    public Limelight(String name) {
        this.name = name;
        limelightData = new ArrayList<LimelightData>();
        tx = 0;
        ty = 0;
        hasTarget = false;
    }

    public double proportionalYaw(double kP) {
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
		// your limelight 3 feed, tx should return roughly 31 degrees.
		double targetingAngularVelocity = getRawTY() * kP;

		// convert to radians per second for our drive method
		targetingAngularVelocity *= Robot.getInstance().getRobotContainer().getSwerveSubsystem().getMaximumAngularVelocity();

		//invert since tx is positive when the target is to the right of the crosshair
		targetingAngularVelocity *= -1.0;

		return targetingAngularVelocity;
    };

    private boolean hasTargetRaw() {
        return LimelightHelpers.getTV(name);
    }
    
    public boolean hasTarget() {
        return hasTarget;
    }

    public String getName() {
        return name;
    }

    public void updateVisionData() {
        limelightData.add(new LimelightData(getRawTX(), getRawTY(), hasTargetRaw()));

        if (limelightData.size() > Constants.LIMELIGHT_DATA_SIZE) {
            limelightData.remove(0);
        }

        double tx_sum = 0;
        double ty_sum = 0;
        double numValid = 0;
        for (LimelightData data : limelightData) {
            tx_sum += data.getTX();
            ty_sum += data.getTY();
            numValid += data.isValid() ? 1 : 0;
        }  

        this.tx = tx_sum / Constants.LIMELIGHT_DATA_SIZE;
        this.ty = ty_sum / Constants.LIMELIGHT_DATA_SIZE;
        hasTarget = numValid >= 3;

        SmartDashboard.putNumber(name + "/processed_tx", tx);
        SmartDashboard.putNumber(name + "/processed_ty", ty);
        SmartDashboard.putBoolean(name + "/has target", hasTarget);
    }

    protected abstract double getRawTX();
    protected abstract double getRawTY();

    public double getTX() {
        return tx;
    }
    
    public double getTY() {
        return ty;
    }
}
