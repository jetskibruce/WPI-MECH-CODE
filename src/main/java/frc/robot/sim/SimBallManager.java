package frc.robot.sim;

import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class SimBallManager {

    private final ArrayList<SimBall> balls = new ArrayList<>();
    private final Field2d field;

    public SimBallManager(Field2d field) {
        this.field = field;

        // Pre-create up to 10 ball objects
        for (int b = 0; b < 10; b++) {
            field.getObject("Ball" + b).setPose(new Pose2d(-5, -5, new Rotation2d()));
        }
    }

    public void spawnBall(Pose2d robotPose, double speed) {
        SimBall ball = new SimBall();
        ball.spawn(robotPose, speed);
        balls.add(ball);
    }

    public void update(double dt) {
        Iterator<SimBall> it = balls.iterator();
        int index = 0;

        while (it.hasNext()) {
            SimBall ball = it.next();
            ball.update(dt);

            if (!ball.isActive()) {
                // Hide the ball
                field.getObject("Ball" + index)
                     .setPose(new Pose2d(-10, -10, new Rotation2d()));
                it.remove();
                continue;
            }

            // Draw the ball as a robot icon
            field.getObject("Ball" + index)
                 .setPose(new Pose2d(ball.getPosition(), ball.getHeading()));

            index++;
        }
    }
}