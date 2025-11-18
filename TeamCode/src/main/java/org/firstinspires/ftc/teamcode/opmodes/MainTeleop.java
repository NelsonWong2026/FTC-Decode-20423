package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "Main Teleop")
public class MainTeleop extends NextFTCOpMode {
    private Drive drive;
    private Intake intake;
    private Shooter shooter;
    private Vision vision;

    private LLResult llResult;
    private Pose3D botPose = new Pose3D(new Position(DistanceUnit.METER, 0, 0, 0, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0,0, 0, 0));
    private JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    //private static final Logger log = LoggerFactory.getLogger(MainTeleop.class);

    @Override
    public void onInit() {
        drive = new Drive(hardwareMap);
        intake = new Intake();
        shooter = new Shooter();
        vision = new Vision(hardwareMap);

        drive.driveCommand(true).schedule();

        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(intake.setIntake())
                .whenBecomesFalse(intake.stopIntake());
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(intake.setOuttake())
                .whenBecomesFalse(intake.stopIntake());
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(shooter.blockBall())
                .whenBecomesFalse(shooter.unblockBall());
        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(new ParallelGroup(
                        //shooter.launchBall()
                        shooter.unblockBall()
                ))
                .whenBecomesFalse(new ParallelGroup(
                        //shooter.resetBallLauncher()
                        shooter.blockBall()
                ));

        vision.startLimelight();
    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {
        vision.updateOrientationWithIMU();
        llResult = vision.getLLResult();
        if (llResult.isValid() && llResult != null) {
            botPose = vision.getLLResult().getBotpose_MT2();
            joinedTelemetry.addData("Distance", llResult.getBotposeAvgDist());
            joinedTelemetry.addData("YDegreesAway", llResult.getTy());
            joinedTelemetry.addData("X Pos", botPose.getPosition().x);
            joinedTelemetry.addData("Y Pos", botPose.getPosition().y);
            joinedTelemetry.addData("Rot", botPose.getOrientation().getYaw());
        }
        joinedTelemetry.addData("BotPose", botPose);
        joinedTelemetry.update();
//        Pose2D botpose = Vision.INSTANCE.getBotPose();
//        telemetry.addData("Bot Pose", "X: %d, Y: %d, Heading: %d",
//                botpose.getX(DistanceUnit.METER), botpose.getY(DistanceUnit.METER), botpose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Top Flywheel Distance", "%.3f rot", shooter.getTopFlywheelDistance());
        telemetry.addData("Bottom Flywheel Distance","%.3f rot", shooter.getBottomFlywheelDistance());
        telemetry.addData("Top Flywheel Velocity","%.3f RPM", shooter.getTopFlywheelVelocity());
        telemetry.addData("Bottom Flywheel Velocity","%.3f RPM", shooter.getBottomFlywheelVelocity());
        telemetry.addData("Pinpoint Position", drive.getPinpointPosition());
        telemetry.update();
    }

    @Override
    public void onStop() {
        vision.stopLimelight();
    }
}
