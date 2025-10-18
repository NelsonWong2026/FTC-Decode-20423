package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.frozenmilk.mercurial.Mercurial;

@Mercurial.Attach
@Shooter.Attach
@Config
@TeleOp(name="ShooterTuning", group = "Tuning")
public class ShooterTuning extends OpMode {
    public static double pos = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        Shooter.INSTANCE.launcherPosition(pos);
        telemetry.update();
    }
}