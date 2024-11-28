package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

/**
 * Represents the speeds of the top and bottom motors of the shooter.
 */
public class ShooterSpeed implements Interpolatable<ShooterSpeed> {
    double topMotorSpeed;
    double bottomMotorSpeed;

    public ShooterSpeed(double topMotorSpeed, double bottomMotorSpeed) {
        this.topMotorSpeed = topMotorSpeed;
        this.bottomMotorSpeed = bottomMotorSpeed;
    }

    private ShooterSpeed diff (ShooterSpeed o) {
        return new ShooterSpeed(
            this.topMotorSpeed - o.topMotorSpeed,
            this.bottomMotorSpeed - o.bottomMotorSpeed
        );
    }

    private ShooterSpeed sum(ShooterSpeed o) {
        return new ShooterSpeed(
            this.topMotorSpeed + o.topMotorSpeed,
            this.bottomMotorSpeed + o.bottomMotorSpeed
        );
    }

    private ShooterSpeed product(double scalar) {
        return new ShooterSpeed(
            this.topMotorSpeed * scalar,
            this.bottomMotorSpeed * scalar
        );
    }

    @Override
    public ShooterSpeed interpolate(ShooterSpeed endValue, double t) {
        ShooterSpeed delta = this.diff(endValue);
        ShooterSpeed interpolated = this.sum(delta.product(t));
        return interpolated;
    }

    /**
     * Returns interpolator for Double.
     *
     * @return Interpolator for Double.
     */
    static Interpolator<ShooterSpeed> getInterpolator() {
        return ShooterSpeed::interpolate;
    }
}