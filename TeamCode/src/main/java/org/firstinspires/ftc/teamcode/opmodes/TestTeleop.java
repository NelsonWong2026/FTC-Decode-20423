package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;

@TeleOp(name = "Test Teleop")
public class TestTeleop extends NextFTCOpMode {
    private LLResult llResult;
    private Pose3D botPose = new Pose3D(new Position(DistanceUnit.METER, 0, 0, 0, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0,0, 0, 0));
    private JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
//private static final Logger log = LoggerFactory.getLogger(MainTeleop.class);

    public TestTeleop() {
        addComponents(
                new SubsystemComponent(Drive.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE, Vision.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {
        MecanumDriverControlled driveControlled = Drive.INSTANCE.driveControlled(true);
        driveControlled.schedule();

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> driveControlled.setScalar(0.4))
                .whenBecomesFalse(() -> driveControlled.setScalar(1));

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(Drive.INSTANCE::zeroPinpoint);

        Gamepads.gamepad1().leftStickButton()
                .whenBecomesTrue(Drive.INSTANCE.enableHeadingPID());
        //.whenBecomesFalse(Drive.INSTANCE.disableHeadingPID());

        Gamepads.gamepad1().leftStickY().asButton(value -> (Math.abs(value) > 0.03)).or(
                        Gamepads.gamepad1().leftStickX().asButton(value -> (Math.abs(value) > 0.03)
                        )).or(
                        Gamepads.gamepad1().rightStickX().asButton(value -> (Math.abs(value) > 0.03)
                        ))
                .whenBecomesTrue(Drive.INSTANCE.disableHeadingPID())
                .whenBecomesFalse(Drive.INSTANCE.stopDrive());

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.setIntake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());
        Gamepads.gamepad1().a()
                .whenBecomesTrue(Shooter.INSTANCE.setOuttake())
                .whenBecomesFalse(Shooter.INSTANCE.stopShooter());
        Gamepads.gamepad1().b()
                .whenBecomesTrue(Shooter.INSTANCE.setIntake())
                .whenBecomesFalse(Shooter.INSTANCE.stopShooter());
        Gamepads.gamepad1().x()
                .whenBecomesTrue(Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.NEAR_SHOOTER_TOP_RPM, Constants.Shooter.NEAR_SHOOTER_BOTTOM_RPM))
                .whenBecomesFalse(() -> {
                    Shooter.INSTANCE.disableFlyWheelPID();
                    Shooter.INSTANCE.stopShooter();
                });
        Gamepads.gamepad1().y()
                .whenBecomesTrue(Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.FAR_SHOOTER_TOP_RPM, Constants.Shooter.FAR_SHOOTER_BOTTOM_RPM))
                .whenBecomesFalse(() -> {
                    Shooter.INSTANCE.disableFlyWheelPID();
                    Shooter.INSTANCE.stopShooter();
                });
        Gamepads.gamepad1().leftTrigger().atLeast(0.2)
                .whenBecomesTrue(Shooter.INSTANCE.unblockBall())
                .whenBecomesFalse(Shooter.INSTANCE.blockBall());
        Gamepads.gamepad1().rightTrigger().atLeast(0.2)
                .whenTrue(Intake.INSTANCE.outtakeWhenFlywheelsReady())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());

        Gamepads.gamepad1().back()
                .whenBecomesTrue(Drive.INSTANCE.enableHeadingPID())
                .whenBecomesFalse(Drive.INSTANCE.disableHeadingPID());

        Vision.INSTANCE.startLimelight();
    }

    @Override
    public void onUpdate() {
        /*llResult = Vision.INSTANCE.getLLResult();
        if (llResult != null && llResult.isValid()) {
            botPose = Vision.INSTANCE.getLLResult().getBotpose_MT2();
            joinedTelemetry.addData("Distance", llResult.getBotposeAvgDist());
            joinedTelemetry.addData("XDegreesAway", llResult.getTx());
            joinedTelemetry.addData("YDegreesAway", llResult.getTy());
            joinedTelemetry.addData("X Pos", botPose.getPosition().x);
            joinedTelemetry.addData("Y Pos", botPose.getPosition().y);
            joinedTelemetry.addData("Rot", botPose.getOrientation().getYaw());
        }*/
        //joinedTelemetry.addData("BotPose", botPose);
//        Pose2D botpose = Vision.INSTANCE.getBotPose();
//        telemetry.addData("Bot Pose", "X: %d, Y: %d, Heading: %d",
//                botpose.getX(DistanceUnit.METER), botpose.getY(DistanceUnit.METER), botpose.getHeading(AngleUnit.DEGREES));
        //joinedTelemetry.addData("Top Flywheel Distance", "%.3f rot", Shooter.INSTANCE.getTopFlywheelDistance());
        //joinedTelemetry.addData("Bottom Flywheel Distance","%.3f rot", Shooter.INSTANCE.getBottomFlywheelDistance());
        joinedTelemetry.addData("Top Flywheel Velocity","%.3f RPM", Shooter.INSTANCE.getTopFlywheelVelocity());
        joinedTelemetry.addData("Bottom Flywheel Velocity","%.3f RPM", Shooter.INSTANCE.getBottomFlywheelVelocity());
        joinedTelemetry.addData("Ready to Shoot: ", Shooter.INSTANCE.flyWheelsWithinVelocityTolerance());
        //joinedTelemetry.addData("Pinpoint Position", Drive.INSTANCE.getPinpointPosition());
        joinedTelemetry.update();
    }

    @Override
    public void onStop() {
        Vision.INSTANCE.stopLimelight();
    }

}
