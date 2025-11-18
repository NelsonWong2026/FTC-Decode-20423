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

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

import static dev.nextftc.bindings.Bindings.*;

@Config
@TeleOp(name="ShooterTuning", group = "Tuning")
public class ShooterTuning extends NextFTCOpMode {
    private Drive drive;
    private Intake intake;
    private Shooter shooter;
    public static double blockerPos = Constants.Shooter.CLEAR_POS;
    public static double shooterVelocity = 0;

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(hardwareMap);
        intake = new Intake();
        shooter = new Shooter();

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
                        //shooter.unblockBall()
                ))
                .whenBecomesFalse(new ParallelGroup(
                        //shooter.resetBallLauncher()
                        //shooter.blockBall()
                ));
    }

    @Override
    public void onUpdate() {
        //shooter.launcherPosition(launcherPos);
        //shooter.blockerPosition(blockerPos);
        shooter.setTargetVelocity(shooterVelocity);
        telemetry.addData("Target Velocity", shooterVelocity);
        telemetry.addData("Top Flywheel Velocity", shooter.getTopFlywheelVelocity());
        telemetry.addData("Bottom Flywheel Velocity", shooter.getBottomFlywheelVelocity());
        telemetry.addData("Pinpoint Position", drive.getPinpointPosition());
        telemetry.update();
    }
}