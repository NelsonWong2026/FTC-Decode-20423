package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Red Auto", group = "Autonomous")
@Configurable // Panels
public class RedAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private boolean shotsTriggered = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.101083032490973, 122.85920577617328, Math.toRadians(143.13)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate();
        Shooter.INSTANCE.update();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain score1;
        public PathChain alignFirstRow;
        public PathChain firstRow;
        public PathChain score2;
        public PathChain alignSecondRow;
        public PathChain secondRow;
        public PathChain lever;
        public PathChain score3;
        public PathChain alignThirdRow;
        public PathChain thirdRow;
        public PathChain score4;
        public PathChain zoneIntake;
        public PathChain score5;
        public PathChain park;

        public Paths(Follower follower) {
            score1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.101, 122.859).mirror(), new Pose(54.585, 88.722).mirror())
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(143.13),
                            Math.toRadians(132)
                    )
                    .build();

            alignFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.585, 88.722).mirror(), new Pose(47.000, 84.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                    .build();

            firstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.000, 84.000).mirror(), new Pose(33.000, 84.000).mirror())
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            score2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.000, 84.000).mirror(), new Pose(54.412, 88.895).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                    .build();

            alignSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.412, 88.895).mirror(), new Pose(47.000, 60.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                    .build();

            secondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.000, 60.000).mirror(), new Pose(23.000, 60.000).mirror())
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            lever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(23.000, 60.000).mirror(),
                                    new Pose(44.2, 65.25).mirror(),
                                    new Pose(27.000, 69.487).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            score3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(27.000, 69.487).mirror(), new Pose(54.412, 88.722).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                    .build();

            alignThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.412, 88.722).mirror(), new Pose(47.000, 35.800).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                    .build();

            thirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.000, 35.800).mirror(), new Pose(23.000, 35.800).mirror())
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            score4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.000, 35.800).mirror(), new Pose(54.412, 88.722).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                    .build();

            zoneIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.412, 88.722).mirror(), new Pose(23.000, 13.511).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(45))
                    .build();

            score5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.000, 13.511).mirror(), new Pose(54.412, 88.895).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(132))
                    .build();

            park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.412, 88.895).mirror(), new Pose(33.000, 88.375).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();
        }
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Shooter.INSTANCE.setBlocker();
                Shooter.INSTANCE.setTargetVelocity(
                        org.firstinspires.ftc.teamcode.Constants.Shooter.AUTO_NEAR_SHOOTER_TOP_RPM,
                        org.firstinspires.ftc.teamcode.Constants.Shooter.AUTO_NEAR_SHOOTER_BOTTOM_RPM);
                follower.followPath(paths.score1, true);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        follower.followPath(paths.alignFirstRow);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.firstRow);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.score2, true);
                    setPathState(4);
                }
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        follower.followPath(paths.alignSecondRow);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.secondRow);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.lever);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.score3);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        follower.followPath(paths.alignThirdRow);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.thirdRow);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.score4);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        follower.followPath(paths.zoneIntake);
                        setPathState(12);
                    }
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.score5);
                    setPathState(13);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        follower.followPath(paths.park);
                        setPathState(14);
                    }
                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

}