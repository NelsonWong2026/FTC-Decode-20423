package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Blue Main Auto")
@Disabled
public class BlueMainAuto extends NextFTCOpMode {
    private final Pose startPose = new Pose(20.1, 122.86, Math.toRadians(143.31)); // Start Pose of robot, facing the goal at a -35 degree angle
    private final Pose scorePose = new Pose(54.41, 88.72, Math.toRadians(132)); // Scoring Pose of robot. It is facing the goal at a -35 degree angle.
    private final Pose alignPickup1Pose = new Pose(44.19, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1Pose = new Pose(14.9, 84, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    //private final Pose scorePose = new Pose(54.41, 88.72, Math.toRadians(132)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose alignPickup2Pose = new Pose(44.01, 60, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(8.32, 60, Math.toRadians(0));
    private final Pose gatePose = new Pose(14.036, 69.48, Math.toRadians(90));
    //private final Pose scorePose = new Pose(54.41, 88.72, Math.toRadians(132));
    private final Pose alignPickup3Pose = new Pose(43.15, 35.87, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(8.5, 35.52, Math.toRadians(0));
    //private final Pose scorePose = new Pose(54.41, 88.72, Math.toRadians(132));
    private final Pose pickup4Pose = new Pose(11.26, 11.264, Math.toRadians(45));
    //private final Pose scorePose = new Pose(54.41, 88.9, Math.toRadians(132));
    private final Pose leavePose = new Pose(14.56, 88.38, Math.toRadians(180));


    public BlueMainAuto() {
        addComponents(
                new SubsystemComponent(Drive.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE, Vision.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }


    private Command autoRoutine() {
        return new SequentialGroup(

        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }

    @Override
    public void onUpdate() {

    }
}
