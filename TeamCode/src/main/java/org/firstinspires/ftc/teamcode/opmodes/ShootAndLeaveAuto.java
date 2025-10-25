package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.MercurialAction;
import org.firstinspires.ftc.teamcode.util.SilkRoad;

import dev.frozenmilk.mercurial.Mercurial;

@Shooter.Attach
@Drive.Attach
@Intake.Attach
@SilkRoad.Attach
@Mercurial.Attach
@Config
@Autonomous(name = "Shoot and Leave Auto", group = "Autonomous")
public class ShootAndLeaveAuto extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void start() {
        SilkRoad.RunAsync(
            new SequentialAction(
                new MercurialAction(Shooter.INSTANCE.setOuttake()),
                new SleepAction(3),
                //shoot ball
                new MercurialAction(Shooter.INSTANCE.launchBall()),
                new SleepAction(1),
                new MercurialAction(Shooter.INSTANCE.resetBallLauncher()),
                new SleepAction(1),
                //intake
                new MercurialAction(Intake.INSTANCE.setIntake()),
                new SleepAction(3),
                new MercurialAction(Intake.INSTANCE.stopIntake()),
                //shoot ball
                new MercurialAction(Shooter.INSTANCE.launchBall()),
                new SleepAction(1),
                new MercurialAction(Shooter.INSTANCE.resetBallLauncher()),
                new SleepAction(1),
                new MercurialAction(Intake.INSTANCE.setIntake()),
                new SleepAction(3),
                new MercurialAction(Intake.INSTANCE.stopIntake()),
                //shoot ball
                new MercurialAction(Shooter.INSTANCE.launchBall()),
                new SleepAction(1),
                new MercurialAction(Shooter.INSTANCE.resetBallLauncher()),
                new SleepAction(1),
                new MercurialAction(Shooter.INSTANCE.stopShooter()),
                //drive forward 0.5 seconds
                new MercurialAction(Drive.INSTANCE.driveForward()),
                new SleepAction(0.5),
                new MercurialAction(Drive.INSTANCE.stopDrive())
            )
        );
    }

    @Override
    public void loop() {

    }
}
