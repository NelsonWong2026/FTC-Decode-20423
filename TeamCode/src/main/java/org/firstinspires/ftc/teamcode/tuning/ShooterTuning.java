package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;

import static dev.nextftc.bindings.Bindings.*;

@Config
@TeleOp(name="ShooterTuning", group = "Tuning")
public class ShooterTuning extends NextFTCOpMode {
    public static double blockerPos = Constants.Shooter.CLEAR_POS;
    public static double shooterVelocity = 0;
    public static double topShooterVelocity = 0;
    public static double bottomShooterVelocity = 0;

    public ShooterTuning() {
        addComponents(
                new SubsystemComponent(Drive.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE, Vision.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void onStartButtonPressed() {
        MecanumDriverControlled driveControlled = Drive.INSTANCE.driveControlled(true);
        driveControlled.schedule();

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> driveControlled.setScalar(0.4))
                .whenBecomesFalse(() -> driveControlled.setScalar(1));

        Gamepads.gamepad1().a()
                .whenBecomesTrue(Drive.INSTANCE::zeroPinpoint);
        Gamepads.gamepad2().x()
                .whenBecomesTrue(Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.FAR_SHOOTER_TOP_RPM, Constants.Shooter.FAR_SHOOTER_BOTTOM_RPM))
                .whenBecomesFalse(Shooter.INSTANCE.disableFlyWheelPID());
        Gamepads.gamepad2().y()
                .whenBecomesTrue(Shooter.INSTANCE.setIntake())
                .whenBecomesFalse(Shooter.INSTANCE.stopShooter());
        Gamepads.gamepad2().a()
                .whenBecomesTrue(Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.NEAR_SHOOTER_TOP_RPM, Constants.Shooter.NEAR_SHOOTER_BOTTOM_RPM))
                .whenBecomesFalse(Shooter.INSTANCE.disableFlyWheelPID());
        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(Intake.INSTANCE.setIntake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(Intake.INSTANCE.setOuttake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(Shooter.INSTANCE.unblockBall())
                .whenBecomesFalse(Shooter.INSTANCE.blockBall());
        /*Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(new ParallelGroup(
                        //Shooter.INSTANCE.launchBall()
                        //Shooter.INSTANCE.unblockBall()
                ))
                .whenBecomesFalse(new ParallelGroup(
                        //Shooter.INSTANCE.resetBallLauncher()
                        //Shooter.INSTANCE.blockBall()
                ));*/
    }

    @Override
    public void onUpdate() {
        //shooter.launcherPosition(launcherPos);
        //shooter.blockerPosition(blockerPos);
        Shooter.INSTANCE.setTopTargetVelocity(topShooterVelocity);
        Shooter.INSTANCE.setBottomTargetVelocity(bottomShooterVelocity);
        telemetry.addData("Target Velocity", shooterVelocity);
        telemetry.addData("Top Flywheel Velocity", Shooter.INSTANCE.getTopFlywheelVelocity());
        telemetry.addData("Bottom Flywheel Velocity", Shooter.INSTANCE.getBottomFlywheelVelocity());
        telemetry.addData("Pinpoint Position", Drive.INSTANCE.getPinpointPosition());
        telemetry.update();
    }
}