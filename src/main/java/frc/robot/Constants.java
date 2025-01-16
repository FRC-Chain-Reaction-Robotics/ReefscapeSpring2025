package frc.robot;

public final class Constants {
    public final class Swerve {
        public static final double WHEEL_RADIUS = 0.1016; // in meters
        public static final double VERTICAL_MODULE_DISTANCE = 0; // TODO: distance from center of bot to forward/rear modules in meters
        public static final double HORIZONTAL_MODULE_DISTANCE = 0; // TODO: distance from center of bot to left/right modules in meters
        public static final double WHEEL_COF = 1.19; // temporary placeholder LOW PRIORITY (DO LAST)
        public static final double MAX_SPEED = 8; // in m/s, TODO: TEMP VALUE
        public static final double DRIVE_RATIO = 6.75; // gear ratio of drive motor (x rotations to 1 wheel rotation)
        public static final double DRIVE_CURRENT_LIM = 40; // in Amps
    }

    public final class Robot {
        public static final double MASS = 0; // TODO: get mass of bot
        public static final double MOMENT_OF_INERTIA = 0; // TODO: in KG*M^2
    }
}
