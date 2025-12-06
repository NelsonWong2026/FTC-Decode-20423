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

@Autonomous(name = "Blue Near Auto 2", group = "Autonomous")
@Configurable // Panels
public class BlueAuto2 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(14.975999999999999, 128.256, Math.toRadians(135)));

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
        pathState = currentPath(); // Update autonomous state machine
        autonomousPathUpdate();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain startBalls;
        public PathChain firstRow;
        public PathChain shootFirstRow;
        public PathChain lever;
        public PathChain secondRow;
        public PathChain shootSecondRow;
        public PathChain thirdRow;
        public PathChain shootThirdRow;

        public Paths(Follower follower) {
            startBalls = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(14.976, 128.256), new Pose(71.808, 71.424))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            firstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(71.808, 71.424),
                                    new Pose(60.672, 83.328),
                                    new Pose(21.504, 83.712)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(21.504, 83.712),
                                    new Pose(59.328, 83.520),
                                    new Pose(71.424, 71.232)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            lever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(71.424, 71.232),
                                    new Pose(67.200, 75.648),
                                    new Pose(16.128, 73.344)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            secondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.128, 73.344),
                                    new Pose(74.112, 61.248),
                                    new Pose(18.816, 59.904)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.816, 59.904),
                                    new Pose(32.064, 60.288),
                                    new Pose(59.520, 83.520),
                                    new Pose(70.080, 70.656)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            thirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(70.080, 70.656),
                                    new Pose(48.960, 94.464),
                                    new Pose(67.776, 35.712),
                                    new Pose(21.696, 35.904)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(21.696, 35.904),
                                    new Pose(33.792, 35.520),
                                    new Pose(48.384, 95.040),
                                    new Pose(70.272, 73.536)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }

    public int currentPath() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.startBalls);
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
                    follower.followPath(paths.firstRow);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.shootFirstRow);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.lever);
                    setPathState(4);
                }
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.secondRow);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.shootSecondRow);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.thirdRow);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.shootThirdRow);
                    setPathState(8);
                }
                break;
            case 9:
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