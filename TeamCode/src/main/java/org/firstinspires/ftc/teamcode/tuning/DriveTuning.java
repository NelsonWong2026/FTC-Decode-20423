package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
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
@TeleOp(name="DriveTuning", group = "Tuning")
public class DriveTuning extends NextFTCOpMode {
    public static boolean headingControl = false;
    private LLResult llResult;
    private MecanumDriverControlled driveControlled = Drive.INSTANCE.driveControlledIMU(true);
    public static double headingGoal = 0.0;

    public DriveTuning() {
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
        driveControlled.schedule();

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> driveControlled.setScalar(0.4))
                .whenBecomesFalse(() -> driveControlled.setScalar(1));

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    Drive.INSTANCE.zeroPinpoint();
                    Drive.INSTANCE.zeroIMU();
                });

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(Intake.INSTANCE.setIntake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(Intake.INSTANCE.setOuttake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());

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

        Gamepads.gamepad1().leftStickButton()
                .whenBecomesTrue(Drive.INSTANCE.enableHeadingPID());
        Gamepads.gamepad1().rightStickButton()
                .whenBecomesTrue(Drive.INSTANCE.disableHeadingPID())
                .whenBecomesFalse(Drive.INSTANCE.stopDrive());

        Gamepads.gamepad1().leftStickY().asButton(value -> (value > 0 || value < 0)).or(
                    Gamepads.gamepad1().leftStickX().asButton(value -> (value > 0 || value < 0)
                )).or(
                    Gamepads.gamepad1().rightStickX().asButton(value -> (value > 0 || value < 0)
                ))
                .whenBecomesTrue(Drive.INSTANCE.disableHeadingPID())
                .whenBecomesFalse(Drive.INSTANCE.stopDrive());

        Vision.INSTANCE.startLimelight();
    }

    @Override
    public void onUpdate() {
        Drive.INSTANCE.setHeadingGoal(headingGoal);
        /*if (headingControl) {
            driveControlled.cancel();
            Drive.INSTANCE.enableHeadingPID();
        }
        else {
            driveControlled.schedule();
            Drive.INSTANCE.disableHeadingPID();
        }*/
        telemetry.addData("Heading Error", Vision.INSTANCE.xCrosshairOffset());
        telemetry.addData("Heading Goal", headingGoal);
        telemetry.addData("Heading Velo", Drive.INSTANCE.getHeadingVelocity());
        telemetry.addData("Distance from Goal", Vision.INSTANCE.DistanceFromGoal());
        telemetry.update();
    }

    @Override
    public void onStop() {
        Vision.INSTANCE.stopLimelight();
    }
}