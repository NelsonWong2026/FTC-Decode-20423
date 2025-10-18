package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import dev.frozenmilk.mercurial.Mercurial;

@Mercurial.Attach
@Drive.Attach
@Intake.Attach
@Shooter.Attach
@Vision.Attach
@TeleOp(name = "Main Teleop")
public class MainTeleop extends OpMode {

    //private static final Logger log = LoggerFactory.getLogger(MainTeleop.class);

    @Override
    public void init() {
        Drive.INSTANCE.setDefaultCommand(Drive.INSTANCE.driveCommand(true));
        Mercurial.gamepad2().dpadUp()
                .onTrue(Intake.INSTANCE.setIntake())
                .onFalse(Intake.INSTANCE.stopIntake());
        Mercurial.gamepad2().dpadDown()
                .onTrue(Intake.INSTANCE.setOuttake())
                .onFalse(Intake.INSTANCE.stopIntake());
        Mercurial.gamepad2().x()
                .onTrue(Shooter.INSTANCE.setIntake())
                .onFalse(Shooter.INSTANCE.stopShooter());
        Mercurial.gamepad2().y()
                .onTrue(Shooter.INSTANCE.setOuttake())
                .onFalse(Shooter.INSTANCE.stopShooter());
        Mercurial.gamepad2().rightBumper()
                .onTrue(Shooter.INSTANCE.launchBall())
                .onFalse(Shooter.INSTANCE.resetBallLauncher());
        //Vision.INSTANCE.startLimelight();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
//        Pose2D botpose = Vision.INSTANCE.getBotPose();
//        telemetry.addData("Bot Pose", "X: %d, Y: %d, Heading: %d",
//                botpose.getX(DistanceUnit.METER), botpose.getY(DistanceUnit.METER), botpose.getHeading(AngleUnit.DEGREES));
//        telemetry.update();
    }

    @Override
    public void stop() {
        //Vision.INSTANCE.stopLimelight();
    }
}
