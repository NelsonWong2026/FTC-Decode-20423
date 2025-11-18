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

import static dev.nextftc.bindings.Bindings.*;

@Config
@TeleOp(name="ShooterTuning", group = "Tuning")
public class ShooterTuning extends NextFTCOpMode {
    public static double blockerPos = Constants.Shooter.CLEAR_POS;
    public static double shooterVelocity = 0;

    {
        addComponents(
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(Drive.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE, Vision.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drive.INSTANCE.driveCommand(true).schedule();

        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(Intake.INSTANCE.setIntake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(Intake.INSTANCE.setOuttake())
                .whenBecomesFalse(Intake.INSTANCE.stopIntake());
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(Shooter.INSTANCE.blockBall())
                .whenBecomesFalse(Shooter.INSTANCE.unblockBall());
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
        Shooter.INSTANCE.setTargetVelocity(shooterVelocity);
        telemetry.addData("Target Velocity", shooterVelocity);
        telemetry.addData("Top Flywheel Velocity", Shooter.INSTANCE.getTopFlywheelVelocity());
        telemetry.addData("Bottom Flywheel Velocity", Shooter.INSTANCE.getBottomFlywheelVelocity());
        telemetry.addData("Pinpoint Position", Drive.INSTANCE.getPinpointPosition());
        telemetry.update();
    }
}