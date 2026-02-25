package frc.robot;

public final class Constants {

    public static final class DriveConstants {
        public static final int FRONT_LEFT_ID = 0;   // PWM ports
        public static final int FRONT_RIGHT_ID = 1;
        public static final int BACK_LEFT_ID = 2;
        public static final int BACK_RIGHT_ID = 3;

        public static final double WHEEL_RADIUS_METERS = 0.0508; // 4 in
        public static final double GEAR_RATIO = 10.71;

        public static final double TRACK_WIDTH_METERS = 0.6;
        public static final double WHEELBASE_METERS = 0.6;
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}