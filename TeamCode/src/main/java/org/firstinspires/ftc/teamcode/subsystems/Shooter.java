package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.*;
import static org.firstinspires.ftc.teamcode.config.ShooterPIDF.BOTTOM_SHOOTER_D;
import static org.firstinspires.ftc.teamcode.config.ShooterPIDF.BOTTOM_SHOOTER_FF;
import static org.firstinspires.ftc.teamcode.config.ShooterPIDF.BOTTOM_SHOOTER_I;
import static org.firstinspires.ftc.teamcode.config.ShooterPIDF.BOTTOM_SHOOTER_P;
import static org.firstinspires.ftc.teamcode.config.ShooterPIDF.TOP_SHOOTER_D;
import static org.firstinspires.ftc.teamcode.config.ShooterPIDF.TOP_SHOOTER_FF;
import static org.firstinspires.ftc.teamcode.config.ShooterPIDF.TOP_SHOOTER_I;
import static org.firstinspires.ftc.teamcode.config.ShooterPIDF.TOP_SHOOTER_P;

import androidx.annotation.NonNull;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.config.ShooterPIDF;

import java.util.function.DoubleSupplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;


public class Shooter implements Subsystem {
    private MotorEx topFlyWheel = new MotorEx(Constants.Shooter.topFlyWheel).floatMode();
    private MotorEx bottomFlyWheel = new MotorEx(Constants.Shooter.bottomFlyWheel).floatMode();
    private ServoEx blocker = new ServoEx(Constants.Shooter.blocker);
    private DoubleSupplier bottomEncoder, topEncoder;

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private double targetVel = 0.0;
    private boolean flywheelsEnabled = false;


    private ControlSystem flywheelControlSystem = ControlSystem.builder()
            .velPid(TOP_SHOOTER_P, TOP_SHOOTER_I, TOP_SHOOTER_D)
            .basicFF(TOP_SHOOTER_FF)
            .build();

    /*private ControlSystem bottomFlywheelControlSystem = ControlSystem.builder()
            .velPid(BOTTOM_SHOOTER_P, BOTTOM_SHOOTER_I, BOTTOM_SHOOTER_D)
            .basicFF(BOTTOM_SHOOTER_FF)
            .build();*/


    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        if (flywheelsEnabled) {
            flywheelControlSystem.setGoal(new KineticState(0.0, targetVel));
            /*topFlywheelControlSystem.setGoal(new KineticState(0.0, targetVel));
            bottomFlywheelControlSystem.setGoal(new KineticState(0.0, targetVel));*/
        }
        else {
            flywheelControlSystem.setGoal(new KineticState(0.0, 0.0));
            /*topFlywheelControlSystem.setGoal(new KineticState(0.0, 0.0));
            bottomFlywheelControlSystem.setGoal(new KineticState(0.0, 0.0));*/
        }

        double power = flywheelControlSystem.calculate(new KineticState(topFlyWheel.getCurrentPosition(), topFlyWheel.getVelocity()));
        topFlyWheel.setPower(power);
        bottomFlyWheel.setPower(power);

        ActiveOpMode.telemetry().addData("targetVelo", targetVel);
        ActiveOpMode.telemetry().addData("flywheel goal", flywheelControlSystem.getGoal());
    }

    public double calculateShooterVelocity(double distance) {
        return Math.sqrt((GRAVITY*Math.pow(distance, 2))
                /   2*Math.pow(Math.cos(Math.toRadians(SHOOTER_ANGLE)), 2)*(distance*Math.tan(Math.toRadians(SHOOTER_ANGLE))-(TARGET_HEIGHT-SHOOTER_HEIGHT)));
    }

    public double calculateRPM(double v) {
        //need to experimentally find
        return v;
    }

    //set Target method
    public void setTargetVelocity(double target) {
        flywheelsEnabled = true;
        this.targetVel = target * shooterTicksPerRevolution / 60; //converting rpm to tick/s
    }

    public double getTopFlywheelVelocity() {
        return topFlyWheel.getVelocity() / shooterTicksPerRevolution * 60;
    }

    public double getBottomFlywheelVelocity() {
        return bottomFlyWheel.getVelocity() / shooterTicksPerRevolution * 60;
    }

    public double getTopFlywheelDistance() {
        return topEncoder.getAsDouble() / shooterTicksPerRevolution;
    }

    public double getBottomFlywheelDistance() {
        return bottomEncoder.getAsDouble() / shooterTicksPerRevolution;
    }

    public void setBlocker() {
        blocker.setPosition(BLOCK_POS);
    }

    public void clearBlocker() {
        blocker.setPosition(CLEAR_POS);
    }

    public void blockerPosition(double pos) {
        blocker.setPosition(pos);
    }

    public void intake() {
        flywheelsEnabled = false;
        topFlyWheel.setPower(-1);
        bottomFlyWheel.setPower(-1);
    }

    public void outtake() {
        flywheelsEnabled = false;
        topFlyWheel.setPower(1);
        bottomFlyWheel.setPower(1);
    }

    public void stop() {
        flywheelsEnabled = false;
        topFlyWheel.setPower(0);
        bottomFlyWheel.setPower(0);
    }

    public LambdaCommand setIntake() {
        return new LambdaCommand()
                .setStart(this::intake);
    }

    public LambdaCommand setOuttake() {
        return new LambdaCommand()
                .setStart(this::outtake);
    }

    public LambdaCommand stopShooter() {
        return new LambdaCommand()
                .setStart(this::stop);
    }

    public LambdaCommand setShooterTargetVelocity(double target) {
        return new LambdaCommand()
                .setStart(() -> setTargetVelocity(target))
                .setIsDone(() -> flywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, 30.0, Double.POSITIVE_INFINITY)));
    }

    public LambdaCommand blockBall() {
        return new LambdaCommand()
                .setStart(this::setBlocker);
    }

    public LambdaCommand unblockBall() {
        return new LambdaCommand()
                .setStart(this::clearBlocker);
    }
}
