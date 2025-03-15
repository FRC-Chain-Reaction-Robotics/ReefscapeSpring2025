package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public final class Swerve {
        public static final double WHEEL_RADIUS = 0.1016; // in meters
        public static final double VERTICAL_MODULE_DISTANCE = Units.inchesToMeters(3.25); // TODO: distance from center of bot to forward/rear modules in meters
        public static final double HORIZONTAL_MODULE_DISTANCE = Units.inchesToMeters(3.25); // TODO: distance from center of bot to left/right modules in meters
        public static final double WHEEL_COF = 1.19;
        public static final double MAX_SPEED = 2.9; // in m/s, TODO: TEMP VALUE
        public static final double DRIVE_RATIO = 6.75; // gear ratio of drive motor (x rotations to 1 wheel rotation)
        public static final double DRIVE_CURRENT_LIM = 40; // in Amps
        public static final double REAL_MAX_SPEED = 0; //TODO: TEST PLEASE I BEG
    }

    public final class Robot {
        public static final double MASS = Units.lbsToKilograms(100); // TODO: get mass of bot
        public static final double MOMENT_OF_INERTIA = 0; // TODO: in KG*M^2
    }

    public final class Coral {
        public static final double kArmMOI = 0.005;
        public static final double kArmGearing = 1;
        public static final double kRaisedPosition = Units.degreesToRadians(110.0);
        public static final double kLoweredPosition = Units.degreesToRadians(0.0);
        public static final int forwardLimitSwitch = -1;
        public static final int reverseLimitSwitch = -1;
    }

    public final class Elevator {
        public static final int forwardLimitSwitch = 9;
        public static final int reverseLimitSwitch = 8;
    }

    public final class Algae {
        public static final int upLimitSwitch = -1;
        public static final int downLimitSwitch = -1;
    }

    public final class Limelight {
        public static final String NAME = "limelight";
    }

    public final class Lift {
        public static final int LIFTER_MOTOR_ID = 25;
        public static final double GEAR_RATIO = 300;
    }
}
