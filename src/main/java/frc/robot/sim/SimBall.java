package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimBall {

    private boolean active = false;
    private double x, y;
    private double vx, vy;
    private Rotation2d heading;

    public void spawn(Pose2d robotPose, double speed) {
        this.x = robotPose.getX();
        this.y = robotPose.getY();
        this.heading = robotPose.getRotation();

        this.vx = speed * heading.getCos();
        this.vy = speed * heading.getSin();

        active = true;
    }

    public void update(double dt) {
        if (!active) return;

        x += vx * dt;
        y += vy * dt;

        // Remove after leaving field
        if (x < 0 || x > 16 || y < 0 || y > 8) {
            active = false;
        }
    }

    public boolean isActive() { return active; }
    public Rotation2d getHeading() { return heading; }
    public edu.wpi.first.math.geometry.Translation2d getPosition() {
        return new edu.wpi.first.math.geometry.Translation2d(x, y);
    }
}