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
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Blue Auto", group = "Autonomous")
@Configurable // Panels
public class BlueAuto extends NextFTCOpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, actionTimer, opmodeTimer, shootTimer;
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private boolean shotsTriggered = false;
    private boolean shooterEnabled = false;
    private double maxPowerWhileIntaking = 0.7;

    public BlueAuto() {
        addComponents(
                new SubsystemComponent(Drive.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE, Vision.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20.799999999999997, 121.42222222222222, Math.toRadians(143.13)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        Shooter.INSTANCE.setBlocker();
        shooterEnabled = true;
        actionTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void onUpdate() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate();
        Shooter.INSTANCE.update();
        if (shooterEnabled == true) {
            Shooter.INSTANCE.enableFlyWheels();
            Shooter.INSTANCE.setTargetVelocity(
                    org.firstinspires.ftc.teamcode.Constants.Shooter.AUTO_NEAR_SHOOTER_TOP_RPM,
                    org.firstinspires.ftc.teamcode.Constants.Shooter.AUTO_NEAR_SHOOTER_BOTTOM_RPM);
        }
        else {
            Shooter.INSTANCE.disableFlyWheels();
            Shooter.INSTANCE.stop();
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("STATE", Shooter.INSTANCE.getFlyWheelState());
        panelsTelemetry.debug("Shots Remaining", Shooter.INSTANCE.getShotsRemaining());
        panelsTelemetry.debug("Intake Position", Intake.INSTANCE.getCurrentPosition());

        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain score1;
        public PathChain alignFirstRow;
        public PathChain firstRow;
        public PathChain score2;
        public PathChain alignSecondRow;
        public PathChain secondRow;
        public PathChain score3;
        public PathChain alignThirdRow;
        public PathChain thirdRow;
        public PathChain score4;
        public PathChain park;

        public Paths(Follower follower) {
            score1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.800, 121.422), new Pose(58.000, 85.700))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(143.13),
                            Math.toRadians(135)
                    )
                    .build();

            alignFirstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 85.700), new Pose(57.956, 82.667))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            firstRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.956, 82.667), new Pose(37.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            score2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(37.000, 84.000), new Pose(58.000, 85.700))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            alignSecondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 85.700), new Pose(54.600, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            secondRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.600, 60.000), new Pose(33.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            score3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(33.000, 60.000),
                                    new Pose(58.489, 58.844),
                                    new Pose(58.000, 85.700)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            alignThirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 85.700), new Pose(54.600, 35.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            thirdRow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.600, 35.800), new Pose(33.000, 35.800))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            score4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.000, 35.800), new Pose(58.000, 85.700))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 85.700), new Pose(36.622, 85.511))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.score1, true);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (!follower.isBusy() && actionTimer.getElapsedTime() > 3) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        shotsTriggered = false;
                        follower.followPath(paths.alignFirstRow, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    Intake.INSTANCE.intake();
                    follower.followPath(paths.firstRow, maxPowerWhileIntaking, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    Intake.INSTANCE.stop();
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
                        shotsTriggered = false;
                        follower.followPath(paths.alignSecondRow, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    Intake.INSTANCE.intake();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.secondRow, maxPowerWhileIntaking, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    Intake.INSTANCE.stop();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.score3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        shotsTriggered = false;
                        follower.followPath(paths.alignThirdRow, true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    Intake.INSTANCE.intake();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.thirdRow, maxPowerWhileIntaking, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    Intake.INSTANCE.stop();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.score4, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        shotsTriggered = false;
                        Intake.INSTANCE.intake();
                        follower.followPath(paths.park, true);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    Intake.INSTANCE.stop();
                    setPathState(-1);
                }
                break;
            /*case 12:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position *//*
                if(!follower.isBusy()) {
                    actionTimer.resetTimer();
                    *//* Grab Sample *//*
                    if (actionTimer.getElapsedTime() > 2) {
                        Intake.INSTANCE.stop();
                        *//* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample *//*
                        follower.followPath(paths.score5);
                        setPathState(13);
                    }
                }
                break;
            case 13:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position *//*
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        Shooter.INSTANCE.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !Shooter.INSTANCE.isBusy()) {
                        shotsTriggered = false;
                        follower.followPath(paths.park);
                        setPathState(14);
                    }
                }
                break;
            case 14:
                *//* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position *//*
                if(!follower.isBusy()) {
                    *//* Set the state to a Case we won't use or define, so it just stops running an new paths *//*
                    setPathState(-1);
                }
                break;*/
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

}