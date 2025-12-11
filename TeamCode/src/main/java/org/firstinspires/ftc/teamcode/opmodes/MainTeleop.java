package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

@TeleOp(name = "Main Teleop")
public class MainTeleop extends NextFTCOpMode {
    private LLResult llResult;
    private Pose3D botPose = new Pose3D(new Position(DistanceUnit.METER, 0, 0, 0, 0),
            new YawPitchRollAngles(AngleUnit.RADIANS, 0,0, 0, 0));
    private JoinedTelemetry joinedTelemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    private ElapsedTime endGameTimer = new ElapsedTime();
    private boolean endGameRumbled = false;
    //private static final Logger log = LoggerFactory.getLogger(MainTeleop.class);

    public MainTeleop() {
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
        endGameTimer.reset();
        MecanumDriverControlled driveControlled = Drive.INSTANCE.driveControlled(true);
        driveControlled.schedule();

        Gamepads.gamepad1().rightBumper()
                        .whenBecomesTrue(() -> driveControlled.setScalar(0.4))
                        .whenBecomesFalse(() -> driveControlled.setScalar(1));

        Gamepads.gamepad1().a()
                        .whenBecomesTrue(Drive.INSTANCE::zeroPinpoint);

        Gamepads.gamepad1().x()
                        .whenBecomesTrue(Drive.INSTANCE.setPinpointPos(new Pose2D(DistanceUnit.METER,
                                67.0*Constants.Drive.coordinateToMeterConversion, -67.75*Constants.Drive.coordinateToMeterConversion,
                                AngleUnit.DEGREES, 0)));

        Gamepads.gamepad1().b()
                .whenBecomesTrue(Drive.INSTANCE.setPinpointPos(new Pose2D(DistanceUnit.METER,
                        67.0*Constants.Drive.coordinateToMeterConversion, 67.75*Constants.Drive.coordinateToMeterConversion,
                        AngleUnit.DEGREES, 0)));

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                        .whenBecomesTrue(Drive.INSTANCE.enableLimelightHeadingStopPID());
                        //.whenBecomesFalse(Drive.INSTANCE.disableHeadingPID());
        Gamepads.gamepad1().leftBumper()
                        .whenBecomesTrue(Drive.INSTANCE.enableLimelightHeadingPID());
        Gamepads.gamepad1().leftStickButton()
                        .whenBecomesTrue(Drive.INSTANCE.enableRedHeadingPID());
        Gamepads.gamepad1().rightStickButton()
                .whenBecomesTrue(Drive.INSTANCE.enableRedHeadingLimelight());
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(Drive.INSTANCE.enableLimelightHeadingStopPID());
        Gamepads.gamepad1().leftStickY().asButton(value -> (Math.abs(value) > 0.03)).or(
                        Gamepads.gamepad1().leftStickX().asButton(value -> (Math.abs(value) > 0.03)
                        )).or(
                        Gamepads.gamepad1().rightStickX().asButton(value -> (Math.abs(value) > 0.03)
                        ))
                .whenBecomesTrue(Drive.INSTANCE.disableHeadingPID())
                .whenBecomesFalse(Drive.INSTANCE.stopDrive());

        /*Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(Vision.INSTANCE.relocalizeWithLimelightCommand());*/


        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(Intake.INSTANCE.setIntake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(Intake.INSTANCE.setOuttake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());
        Gamepads.gamepad2().a()
                .whenBecomesTrue(Shooter.INSTANCE.setOuttake())
                .whenBecomesFalse(Shooter.INSTANCE.stopShooter());
        Gamepads.gamepad2().b()
                .whenBecomesTrue(Shooter.INSTANCE.setIntake())
                .whenBecomesFalse(Shooter.INSTANCE.stopShooter());
        Gamepads.gamepad2().x()
                .whenBecomesTrue(Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.NEAR_SHOOTER_TOP_RPM, Constants.Shooter.NEAR_SHOOTER_BOTTOM_RPM));
                /*.whenBecomesFalse(() -> {
                    Shooter.INSTANCE.disableFlyWheelPID();
                    Shooter.INSTANCE.stopShooter();
                });*/
        Gamepads.gamepad2().y()
                .whenBecomesTrue(Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.FAR_SHOOTER_TOP_RPM, Constants.Shooter.FAR_SHOOTER_BOTTOM_RPM));
        /*        .whenBecomesFalse(() -> {
                    Shooter.INSTANCE.disableFlyWheelPID();
                    Shooter.INSTANCE.stopShooter();
                });*/

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(Shooter.INSTANCE.unblockBall())
                .whenBecomesFalse(Shooter.INSTANCE.blockBall());
        Gamepads.gamepad2().rightBumper()
                .whenTrue(Intake.INSTANCE.outtakeWhenFlywheelsReady())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());

        Vision.INSTANCE.startLimelight();
    }

    @Override
    public void onUpdate() {
        Vision.INSTANCE.updateOrientationWithPinpoint();
        if (endGameTimer.seconds() > 100 && !endGameRumbled) {
            endGameRumbled = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }
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
        joinedTelemetry.update();
//        Pose2D botpose = Vision.INSTANCE.getBotPose();
//        telemetry.addData("Bot Pose", "X: %d, Y: %d, Heading: %d",
//                botpose.getX(DistanceUnit.METER), botpose.getY(DistanceUnit.METER), botpose.getHeading(AngleUnit.DEGREES));
        //joinedTelemetry.addData("Top Flywheel Distance", "%.3f rot", Shooter.INSTANCE.getTopFlywheelDistance());
        //joinedTelemetry.addData("Bottom Flywheel Distance","%.3f rot", Shooter.INSTANCE.getBottomFlywheelDistance());
        joinedTelemetry.addData("Top Flywheel Velocity","%.3f RPM", Shooter.INSTANCE.getTopFlywheelVelocity());
        joinedTelemetry.addData("Bottom Flywheel Velocity","%.3f RPM", Shooter.INSTANCE.getBottomFlywheelVelocity());
        joinedTelemetry.addData("Ready to Shoot: ", Shooter.INSTANCE.flyWheelsWithinVelocityTolerance());
        joinedTelemetry.addData("Bot Heading", Drive.INSTANCE.getPinpointHeadingDeg());
        joinedTelemetry.addData("Bot Pos", Drive.INSTANCE.getPinpointPosition());
        joinedTelemetry.addData("Angle TO Shoot Odom: ", Drive.INSTANCE.angleToFaceRedGoalOdometry());
        joinedTelemetry.addData("Area > 0.1", Vision.INSTANCE.aprilTagAreaGreaterThan(0.1));
        try {
            joinedTelemetry.addData("Angle To Shoot: ", Vision.INSTANCE.angleToFaceGoalLimelight());
            joinedTelemetry.addData("bot pose x", Vision.INSTANCE.getBotPose().getPosition().x*Constants.Drive.meterToCoordinateConversion);
            joinedTelemetry.addData("bot pose y", Vision.INSTANCE.getBotPose().getPosition().y*Constants.Drive.meterToCoordinateConversion);
        } catch (Exception e) {}
        joinedTelemetry.addData("bot pose", Vision.INSTANCE.getBotPose());
        joinedTelemetry.addData("Red goal pose", Constants.Shooter.RED_GOAL_POSE);
        //joinedTelemetry.addData("Pinpoint Position", Drive.INSTANCE.getPinpointPosition());
        joinedTelemetry.update();
    }

    @Override
    public void onStop() {
        Vision.INSTANCE.stopLimelight();
    }
}
