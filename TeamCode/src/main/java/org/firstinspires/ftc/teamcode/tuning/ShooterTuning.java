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

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;

@Mercurial.Attach
@Shooter.Attach
@Intake.Attach
@Drive.Attach
@Config
@TeleOp(name="ShooterTuning", group = "Tuning")
public class ShooterTuning extends OpMode {
    public static double launcherPos = Constants.Shooter.FLAT_POS;
    public static double blockerPos = Constants.Shooter.CLEAR_POS;
    public static double shooterVelocity = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drive.INSTANCE.setDefaultCommand(Drive.INSTANCE.driveCommand(true));
        Mercurial.gamepad2().dpadUp()
                .onTrue(Intake.INSTANCE.setIntake())
                .onFalse(Intake.INSTANCE.stopIntake());
        Mercurial.gamepad2().dpadDown()
                .onTrue(Intake.INSTANCE.setOuttake())
                .onFalse(Intake.INSTANCE.stopIntake());
        Mercurial.gamepad2().leftBumper()
                .onTrue(Shooter.INSTANCE.blockBall())
                .onFalse(Shooter.INSTANCE.unblockBall());
        Mercurial.gamepad2().rightBumper()
                .onTrue(new Parallel(
                        Shooter.INSTANCE.launchBall()
                        //Shooter.INSTANCE.unblockBall()
                ))
                .onFalse(new Parallel(
                        Shooter.INSTANCE.resetBallLauncher()
                        //Shooter.INSTANCE.blockBall()
                ));
    }

    @Override
    public void loop() {
        //Shooter.INSTANCE.launcherPosition(launcherPos);
        //Shooter.INSTANCE.blockerPosition(blockerPos);
        Shooter.INSTANCE.setTargetVelocity(shooterVelocity);
        telemetry.addData("Target Velocity", shooterVelocity);
        telemetry.addData("Top Flywheel Velocity", Shooter.INSTANCE.getTopFlywheelVelocity());
        telemetry.addData("Bottom Flywheel Velocity", Shooter.INSTANCE.getBottomFlywheelVelocity());
        telemetry.update();
    }
}