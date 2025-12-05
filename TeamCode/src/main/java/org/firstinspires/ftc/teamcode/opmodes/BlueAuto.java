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

@Autonomous(name = "Blue Auto", group = "Autonomous")
@Configurable // Panels
public class BlueAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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
                            new BezierLine(new Pose(20.101, 122.859), new Pose(54.585, 88.722))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(143.13),
                            Math.toRadians(132)
                    )
                    .build();

            alignFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.585, 88.722), new Pose(44.188, 84.043))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                    .build();

            firstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.188, 84.043), new Pose(14.903, 83.870))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            score2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.903, 83.870), new Pose(54.412, 88.895))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                    .build();

            alignSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.412, 88.895), new Pose(44.014, 59.957))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                    .build();

            secondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(44.014, 59.957), new Pose(8.318, 59.610))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            lever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8.318, 59.610),
                                    new Pose(33.444, 65.329),
                                    new Pose(14.036, 69.487)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            score3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.036, 69.487), new Pose(54.412, 88.722))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                    .build();

            alignThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.412, 88.722), new Pose(43.148, 35.870))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(0))
                    .build();

            thirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.148, 35.870), new Pose(8.491, 35.523))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            score4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8.491, 35.523), new Pose(54.412, 88.722))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                    .build();

            zoneIntake = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.412, 88.722), new Pose(11.264, 11.264))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(45))
                    .build();

            score5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(11.264, 11.264), new Pose(54.412, 88.895))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(132))
                    .build();

            park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.412, 88.895), new Pose(14.556, 88.375))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                    .build();
        }
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.score1);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.firstRow,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.alignFirstRow,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.firstRow, true);
                    setPathState(4);
                }
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.score2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.alignSecondRow,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.secondRow,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.lever, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.score3, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.alignThirdRow, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.thirdRow, true);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.score4, true);
                    setPathState(12);
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.zoneIntake, true);
                    setPathState(13);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.score5, true);
                    setPathState(14);
                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.park, true);
                    setPathState(15);
                }
                break;
            case 15:
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
    }

}