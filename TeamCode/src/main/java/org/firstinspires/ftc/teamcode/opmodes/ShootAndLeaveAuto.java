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
    private Command scheduledRoutine;

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
                Intake.INSTANCE.setIntake(),
                new Delay(0.1),
                Intake.INSTANCE.stopIntake(),
                new Delay(0.5),
                Intake.INSTANCE.setIntake(),
                new Delay(0.1),
                Intake.INSTANCE.stopIntake(),
                new Delay(0.5),
                Intake.INSTANCE.setIntake(),
                new Delay(0.2),
                Intake.INSTANCE.stopIntake(),
                Shooter.INSTANCE.disableFlyWheelPID(),
                Shooter.INSTANCE.stopShooter(),
                Shooter.INSTANCE.blockBall(),
                Drive.INSTANCE.setDrive(0.4),
                new Delay(0.5),
                Drive.INSTANCE.stopDrive()
        );
    }

    /*private Command redAutoRoutine() {
        return new SequentialGroup(
                Drive.INSTANCE.setHeadingGoalCommand(Constants.Shooter.redFarGoalAutoAngleOffset),
                Shooter.INSTANCE.blockBall(),
                Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.AUTO_FAR_SHOOTER_TOP_RPM, Constants.Shooter.AUTO_FAR_SHOOTER_BOTTOM_RPM),
                new Delay(4),
                Shooter.INSTANCE.unblockBall(),
                new Delay(2),
                Intake.INSTANCE.setIntake(),
                new Delay(0.1),
                Intake.INSTANCE.stopIntake(),
                new Delay(0.5),
                Intake.INSTANCE.setIntake(),
                new Delay(0.1),
                Intake.INSTANCE.stopIntake(),
                new Delay(0.5),
                Intake.INSTANCE.setIntake(),
                new Delay(0.2),
                Intake.INSTANCE.stopIntake(),
                Drive.INSTANCE.disableHeadingPID(),
                Drive.INSTANCE.stopDrive(),
                Shooter.INSTANCE.disableFlyWheelPID(),
                Shooter.INSTANCE.stopShooter(),
                Shooter.INSTANCE.blockBall(),
                Drive.INSTANCE.setDrive(0.4),
                new Delay(0.5),
                Drive.INSTANCE.stopDrive()
        );
    }

    private Command blueAutoRoutine() {
        return new SequentialGroup(
                Drive.INSTANCE.setHeadingGoalCommand(Constants.Shooter.blueFarGoalAutoAngleOffset),
                Shooter.INSTANCE.blockBall(),
                Shooter.INSTANCE.setShooterTargetVelocity(Constants.Shooter.AUTO_FAR_SHOOTER_TOP_RPM, Constants.Shooter.AUTO_FAR_SHOOTER_BOTTOM_RPM),
                new Delay(4),
                Shooter.INSTANCE.unblockBall(),
                new Delay(2),
                Intake.INSTANCE.setIntake(),
                new Delay(0.1),
                Intake.INSTANCE.stopIntake(),
                new Delay(0.5),
                Intake.INSTANCE.setIntake(),
                new Delay(0.1),
                Intake.INSTANCE.stopIntake(),
                new Delay(0.5),
                Intake.INSTANCE.setIntake(),
                new Delay(0.2),
                Intake.INSTANCE.stopIntake(),
                Drive.INSTANCE.disableHeadingPID(),
                Drive.INSTANCE.stopDrive(),
                Shooter.INSTANCE.disableFlyWheelPID(),
                Shooter.INSTANCE.stopShooter(),
                Shooter.INSTANCE.blockBall(),
                Drive.INSTANCE.setDrive(0.4),
                new Delay(0.5),
                Drive.INSTANCE.stopDrive()
        );
    }*/

    @Override
    public void onInit() {
        scheduledRoutine = autoRoutine();
    }

    @Override
    public void onWaitForStart() {
        /*if (gamepad1.x || gamepad2.x) {
            scheduledRoutine = blueAutoRoutine();
        }
        else if (gamepad1.y || gamepad2.y) {
            scheduledRoutine = redAutoRoutine();
        }
        else if (gamepad1.a || gamepad2.a) {
            scheduledRoutine = autoRoutine();
        }
        telemetry.addData("Scheduled Routine: ", scheduledRoutine);
        telemetry.update();*/
    }

    @Override
    public void onStartButtonPressed() {
        scheduledRoutine.schedule();
    }

    @Override
    public void onUpdate() {

    }
}
