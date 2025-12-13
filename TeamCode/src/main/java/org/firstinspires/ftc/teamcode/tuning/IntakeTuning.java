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
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;

import static dev.nextftc.bindings.Bindings.*;

@Config
@TeleOp(name="IntakeTuning", group = "Tuning")
public class IntakeTuning extends NextFTCOpMode {
    public static double intakeGoal = 0.0;

    public IntakeTuning() {
        addComponents(
                new SubsystemComponent(Drive.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE, Vision.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Intake.INSTANCE.enableIntakePID();
    }

    @Override
    public void onStartButtonPressed() {
        MecanumDriverControlled driveControlled = Drive.INSTANCE.driveControlled(true);
        driveControlled.schedule();

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> driveControlled.setScalar(0.4))
                .whenBecomesFalse(() -> driveControlled.setScalar(1));
    }

    @Override
    public void onUpdate() {
        //shooter.launcherPosition(launcherPos);
        //shooter.blockerPosition(blockerPos);
        Intake.INSTANCE.setTargetPosition(intakeGoal);
        telemetry.addData("Target Pos", intakeGoal);
        telemetry.addData("Intake Pos", Intake.INSTANCE.getCurrentPosition());
        telemetry.update();
    }
}