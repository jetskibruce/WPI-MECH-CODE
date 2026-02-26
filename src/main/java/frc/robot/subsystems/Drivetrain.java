package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    // Motors (PWM Spark)
    private final Spark frontLeft  = new Spark(Constants.DriveConstants.FRONT_LEFT_ID);
    private final Spark frontRight = new Spark(Constants.DriveConstants.FRONT_RIGHT_ID);
    private final Spark backLeft   = new Spark(Constants.DriveConstants.BACK_LEFT_ID);
    private final Spark backRight  = new Spark(Constants.DriveConstants.BACK_RIGHT_ID);

    private final MecanumDrive drive =
        new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

    // Field visualization
    private final Field2d field = new Field2d();


    // Kinematics
    private final MecanumDriveKinematics kinematics =
        new MecanumDriveKinematics(
            new Translation2d(Constants.DriveConstants.WHEELBASE_METERS / 2.0,
                              Constants.DriveConstants.TRACK_WIDTH_METERS / 2.0),   // FL
            new Translation2d(Constants.DriveConstants.WHEELBASE_METERS / 2.0,
                             -Constants.DriveConstants.TRACK_WIDTH_METERS / 2.0),   // FR
            new Translation2d(-Constants.DriveConstants.WHEELBASE_METERS / 2.0,
                               Constants.DriveConstants.TRACK_WIDTH_METERS / 2.0),  // BL
            new Translation2d(-Constants.DriveConstants.WHEELBASE_METERS / 2.0,
                             -Constants.DriveConstants.TRACK_WIDTH_METERS / 2.0)    // BR
        );

    // Odometry
    private final MecanumDriveOdometry odometry =
        new MecanumDriveOdometry(
            kinematics,
            new Rotation2d(),
            new MecanumDriveWheelPositions(0, 0, 0, 0),
            new Pose2d()
        );

    // Simple sim state: wheel rotations (fake physics)
    private double flSimRotations = 0.0;
    private double frSimRotations = 0.0;
    private double blSimRotations = 0.0;
    private double brSimRotations = 0.0;

    // Last commanded chassis speeds (for sim)
    private double lastYSpeed = 0.0; // forward/back (m/s-ish)
    private double lastXSpeed = 0.0; // strafe (m/s-ish)
    private double lastRot    = 0.0; // rad/s-ish

    private double simHeadingRadians = 0.0;
    public Drivetrain() {
        // Register with the scheduler
        this.register();

        frontRight.setInverted(true);
        backRight.setInverted(true);

        SmartDashboard.putData("Field", field);

    }

    // Teleop drive (joystick)
    public void drive(double ySpeed, double xSpeed, double rot) {
        // Store for simulation
        lastYSpeed = ySpeed;
        lastXSpeed = xSpeed;
        lastRot    = rot;

        // WPILib convention: forward is -Y
        drive.driveCartesian(xSpeed, ySpeed, rot);

        System.out.println("drive() y=" + ySpeed + " x=" + xSpeed + " rot=" + rot);
    }

    public Field2d getField() {
        return field;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Helpers
    private double simRotationsToMeters(double rotations) {
        double wheelCircumference = 2.0 * Math.PI * Constants.DriveConstants.WHEEL_RADIUS_METERS;
        return rotations * wheelCircumference / Constants.DriveConstants.GEAR_RATIO;
    }

    @Override
    public void periodic() {
        field.setRobotPose(odometry.getPoseMeters());
    }
    @Override
    public void simulationPeriodic() {

        // System.out.println("Drivetrain.simulationPeriodic() running");

        final double dt = 0.02;

        // Treat joystick inputs as chassis speeds (scaled)
        // Scale to something visible in sim
        double maxSpeedMetersPerSecond = 6.0;
        double maxRotRadiansPerSecond  =  Math.PI; // 180 deg/s

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            -lastYSpeed * maxSpeedMetersPerSecond,   // forward
            lastXSpeed * maxSpeedMetersPerSecond,    // strafe
            lastRot * maxRotRadiansPerSecond         // rotation
        );
        // Integrate heading (simple physics)
        simHeadingRadians += chassisSpeeds.omegaRadiansPerSecond * dt;

        // Keep heading in [-pi, pi]
        simHeadingRadians = Math.atan2(Math.sin(simHeadingRadians), Math.cos(simHeadingRadians));

        // Convert chassis speeds to wheel speeds
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        double wheelCircumference = 2.0 * Math.PI * Constants.DriveConstants.WHEEL_RADIUS_METERS;

        // Integrate wheel rotations
        flSimRotations += (wheelSpeeds.frontLeftMetersPerSecond  / wheelCircumference) * dt;
        frSimRotations += (wheelSpeeds.frontRightMetersPerSecond / wheelCircumference) * dt;
        blSimRotations += (wheelSpeeds.rearLeftMetersPerSecond   / wheelCircumference) * dt;
        brSimRotations += (wheelSpeeds.rearRightMetersPerSecond  / wheelCircumference) * dt;

        // Build wheel positions in meters
        MecanumDriveWheelPositions simPositions = new MecanumDriveWheelPositions(
            simRotationsToMeters(flSimRotations),
            simRotationsToMeters(frSimRotations),
            simRotationsToMeters(blSimRotations),
            simRotationsToMeters(brSimRotations)
        );

        // Zero heading (no gyro yet)
        odometry.update(new Rotation2d(simHeadingRadians), simPositions);

        field.setRobotPose(odometry.getPoseMeters());

    }
}