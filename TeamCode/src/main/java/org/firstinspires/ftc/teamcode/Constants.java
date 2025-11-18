package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class Drive {
        public static final String leftFront = "leftFront";
        public static final String leftBack = "leftBack";
        public static final String rightFront = "rightFront";
        public static final String rightBack = "rightBack";
        public static final String pinpoint = "pinpoint";
        public static final double odomXOffset = -7.25;
        public static final double odomYOffset = -4.8;
    }

    public static final class Shooter {
        public static final String topFlyWheel = "topFlyWheel";
        public static final String bottomFlyWheel = "bottomFlyWheel";
        public static final String leftPivot = "leftPivot";
        public static final String rightPivot = "rightPivot";
        public static final String launcher = "launcher";
        public static final String blocker = "blocker";
        public static final double SHOOTER_HEIGHT = 0;
        public static final double TARGET_HEIGHT = 0;
        public static final double FLYWHEEL_RADIUS = 2; //inches
        public static final double GRAVITY = 9.80665;
        public static final double SHOOTER_ANGLE = 40;
        public static final double shooterTicksPerRevolution = 28;

        //launcher servo positions
        public static final double LAUNCH_POS = 0.7;
        public static final double FLAT_POS = 1;

        //blocker servo positions
        public static final double BLOCK_POS = 0;
        public static final double CLEAR_POS = 0.27;

        public static final double PIVOT_P = 0.0;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.0;
    }

    public static final class Intake {
        public static final String intake = "intake";
        public static final double intakeSpeed = 0.8;

        public static final double INTAKE_P = 0.0;
        public static final double INTAKE_I = 0.0;
        public static final double INTAKE_D = 0.0;
    }

    public static final class Vision {
        public static final String limelight = "limelight";
        public static final double llHeight = 0;
        public static final double llAngleAboveGround = 0;
        public static final double aprilTagHeight = 0;
    }
}
