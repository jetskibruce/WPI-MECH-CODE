package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sim.SimBallManager;

public class Indexer extends SubsystemBase {

    private final Drivetrain drivetrain;
    private final SimBallManager ballManager;

    private boolean ballLoaded = true;
    private double respawnTimer = 0.0;

    public Indexer(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // BallManager needs Field2d from drivetrain
        this.ballManager = new SimBallManager(drivetrain.getField());
    }

    // -------------------------
    // INDEXER MOTOR SIMULATION
    // -------------------------
    private double indexerSpeed = 0.0;

    public void feed() {
        indexerSpeed = 1.0;
    }

    public void stop() {
        indexerSpeed = 0.0;
    }

    // -------------------------
    // BALL FIRING LOGIC
    // -------------------------
    public void tryFireBall(boolean shooterReady, boolean triggerPressed) {

        if (triggerPressed && shooterReady && ballLoaded) {

            // Fire ball from robot pose
            Pose2d pose = drivetrain.getPose();
            ballManager.spawnBall(drivetrain.getPose(), 10.0);
            // Consume the ball
            ballLoaded = false;
            respawnTimer = 0.2; // 200 ms delay
        }
    }

    // -------------------------
    // SIMULATION LOOP
    // -------------------------
    @Override
    public void simulationPeriodic() {
        double dt = 0.02;

        // Update ball flight
        ballManager.update(0.02);           

        // Handle ball respawn timer
        if (!ballLoaded) {
            respawnTimer -= dt;
            if (respawnTimer <= 0) {
                ballLoaded = true;
            }
        }
    }
}