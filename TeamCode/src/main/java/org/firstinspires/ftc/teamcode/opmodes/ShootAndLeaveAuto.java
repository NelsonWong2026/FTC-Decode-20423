package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Config
@Autonomous(name = "Shoot and Leave Auto", group = "Autonomous")
public class ShootAndLeaveAuto extends NextFTCOpMode {

    public ShootAndLeaveAuto() {
        addComponents(
                new SubsystemComponent(Drive.INSTANCE, Intake.INSTANCE, Shooter.INSTANCE, Vision.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    private Command autoRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.blockBall(),
                Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.AUTO_FAR_SHOOTER_TOP_RPM, Constants.Shooter.AUTO_FAR_SHOOTER_BOTTOM_RPM),
                new Delay(4),
                Shooter.INSTANCE.unblockBall(),
                new Delay(2),
                Intake.INSTANCE.outtakeWhenFlywheelsReady(),
                new Delay(5),
                Intake.INSTANCE.stopIntake(),
                Shooter.INSTANCE.disableFlyWheelPID(),
                Shooter.INSTANCE.stopShooter(),
                Shooter.INSTANCE.blockBall(),
                Drive.INSTANCE.setDrive(0.4),
                new Delay(0.5),
                Drive.INSTANCE.stopDrive()
        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }

    @Override
    public void onUpdate() {

    }
}
