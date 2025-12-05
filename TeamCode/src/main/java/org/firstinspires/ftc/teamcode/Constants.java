package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import dev.nextftc.core.units.Distance;

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
        public static final double SHOOTER_HEIGHT = Distance.fromIn(14).inMeters;
        public static final double TARGET_HEIGHT = Distance.fromIn(40).inMeters;
        public static final double FLYWHEEL_RADIUS = 2; //inches
        public static final double GRAVITY = 9.80665;
        public static final double SHOOTER_ANGLE = 40;
        public static final double shooterTicksPerRevolution = 28;

        //blocker servo positions
        public static final double BLOCK_POS = 0.11;
        public static final double CLEAR_POS = 0.41;

        public static final double AUTO_FAR_SHOOTER_TOP_RPM = 1400;
        public static final double AUTO_FAR_SHOOTER_BOTTOM_RPM = 1600;

        public static final double FAR_SHOOTER_TOP_RPM = 1400;//1400;
        public static final double FAR_SHOOTER_BOTTOM_RPM = 1600;//1900;

        public static final double NEAR_SHOOTER_TOP_RPM = 1300;
        public static final double NEAR_SHOOTER_BOTTOM_RPM = 1400;

        public static final Pose2D RED_GOAL_POSE = new Pose2D(DistanceUnit.METER,-1.4827, 1.4133, AngleUnit.DEGREES, -53.13);
        public static final Pose2D BLUE_GOAL_POSE = new Pose2D(DistanceUnit.METER,-1.4827, -1.4133, AngleUnit.DEGREES, 53.13);

        public static final double redFarGoalAutoAngleOffset = -3.7;
        public static final double blueFarGoalAutoAngleOffset = 3.7;
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
        public static final double llForward = Distance.fromIn(6.5).inMeters;
        public static final double llHeight = Distance.fromIn(14).inMeters;;
        public static final double llAngleAboveGround = Distance.fromIn(6).inMeters;;
        public static final double goalHeight = Distance.fromIn(40).inMeters;;
        public static final double aprilTagHeight = Distance.fromIn(29.5).inMeters;
    }
}
