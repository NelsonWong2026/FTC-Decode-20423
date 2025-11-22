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

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.config.ShooterPIDF;

import java.util.function.DoubleSupplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.control.feedforward.FeedforwardElement;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Shooter implements Subsystem {
    public static PIDCoefficients topFlyWheelPID = new PIDCoefficients(
            TOP_SHOOTER_P, TOP_SHOOTER_I, TOP_SHOOTER_D
    );
    public static PIDCoefficients bottomFlyWheelPID = new PIDCoefficients(
            BOTTOM_SHOOTER_P, BOTTOM_SHOOTER_I, BOTTOM_SHOOTER_D
    );

    public static BasicFeedforwardParameters topFlyWheelFeedforward = new BasicFeedforwardParameters(TOP_SHOOTER_FF);
    public static BasicFeedforwardParameters bottomFlyWheelFeedforward = new BasicFeedforwardParameters(BOTTOM_SHOOTER_FF);

    public static final Shooter INSTANCE = new Shooter();
    private Shooter() {};

    private MotorEx topFlyWheel = new MotorEx(Constants.Shooter.topFlyWheel).floatMode();
    private MotorEx bottomFlyWheel = new MotorEx(Constants.Shooter.bottomFlyWheel).floatMode().reversed();
    private ServoEx blocker = new ServoEx(Constants.Shooter.blocker);

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private double targetVel = 0.0;
    private double topTargetVel = 0.0;
    private double bottomTargetVel = 0.0;
    private boolean flywheelsEnabled = false;

    private ControlSystem flywheelControlSystem = ControlSystem.builder()
            .velPid(topFlyWheelPID)
            .basicFF(topFlyWheelFeedforward)
            .build();

    private ControlSystem topFlywheelControlSystem = ControlSystem.builder()
            .velPid(topFlyWheelPID)
            .basicFF(topFlyWheelFeedforward)
            .build();

    private ControlSystem bottomFlywheelControlSystem = ControlSystem.builder()
            .velPid(bottomFlyWheelPID)
            .basicFF(bottomFlyWheelFeedforward)
            .build();


    @Override
    public void initialize() {
        // initialization logic (runs on init)
        /*private DcMotorEx topWheel = ActiveOpMode.hardwareMap().get(DcMotorEx.class, Constants.Shooter.topFlyWheel);
        private DcMotorEx bottomWheel = ActiveOpMode.hardwareMap().get(DcMotorEx.class, Constants.Shooter.bottomFlyWheel);
        topWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        //flywheelControlSystem.setGoal(new KineticState(0.0, targetVel));
        if (flywheelsEnabled) {
            topFlywheelControlSystem.setGoal(new KineticState(0.0, topTargetVel));
            bottomFlywheelControlSystem.setGoal(new KineticState(0.0, bottomTargetVel));
            topFlyWheel.setPower(topFlywheelControlSystem.calculate(new KineticState(topFlyWheel.getCurrentPosition(), topFlyWheel.getVelocity())));
            bottomFlyWheel.setPower(bottomFlywheelControlSystem.calculate(new KineticState(-bottomFlyWheel.getCurrentPosition(), -bottomFlyWheel.getVelocity())));
        }
        else {
            //flywheelControlSystem.setGoal(new KineticState(0.0, 0.0));
            topFlywheelControlSystem.setGoal(new KineticState(0.0, 0.0));
            bottomFlywheelControlSystem.setGoal(new KineticState(0.0, 0.0));
        }

        /*double power = flywheelControlSystem.calculate(new KineticState(topFlyWheel.getCurrentPosition(), topFlyWheel.getVelocity()));
        bottomFlyWheel.setPower(power);*/

        //ActiveOpMode.telemetry().addData("targetVelo", targetVel);
        //ActiveOpMode.telemetry().addData("flywheel goal", flywheelControlSystem.getGoal());
        //ActiveOpMode.telemetry().addData("top flywheel goal", topFlywheelControlSystem.getGoal());
        //ActiveOpMode.telemetry().addData("bottom flywheel goal", bottomFlywheelControlSystem.getGoal());
    }

    public double calculateShooterVelocity(double distance) {
        return Math.sqrt((GRAVITY*Math.pow(distance, 2))
                /   2*Math.pow(Math.cos(Math.toRadians(SHOOTER_ANGLE)), 2)*(distance*Math.tan(Math.toRadians(SHOOTER_ANGLE))-(TARGET_HEIGHT-SHOOTER_HEIGHT)));
    }

    public double calculateRPM(double v) {
        //need to experimentally find
        return v;
    }

    /*public void setVelocity(double velocity) {
        topWheel.setVelocity(velocity);
        bottomWheel.setVelocity(velocity);
    }*/

    //set Target method
    public void setTargetVelocity(double target) {
        flywheelsEnabled = true;
        this.targetVel = target * shooterTicksPerRevolution / 60; //converting rpm to tick/s
    }

    public void setTargetVelocity(double topTarget, double bottomTarget) {
        flywheelsEnabled = true;
        this.topTargetVel = topTarget * shooterTicksPerRevolution / 60; //converting rpm to tick/s
        this.bottomTargetVel = bottomTarget * shooterTicksPerRevolution / 60; //converting rpm to tick/s
    }

    public void setTopTargetVelocity(double target) {
        flywheelsEnabled = true;
        this.topTargetVel = target * shooterTicksPerRevolution / 60; //converting rpm to tick/s
    }

    public void setBottomTargetVelocity(double target) {
        flywheelsEnabled = true;
        this.bottomTargetVel = target * shooterTicksPerRevolution / 60; //converting rpm to tick/s
    }

    public double getTopFlywheelVelocity() {
        return topFlyWheel.getVelocity() / shooterTicksPerRevolution * 60;
    }

    public double getBottomFlywheelVelocity() {
        return bottomFlyWheel.getVelocity() / shooterTicksPerRevolution * 60;
    }

    public double getTopFlywheelDistance() {
        return topFlyWheel.getCurrentPosition() / shooterTicksPerRevolution;
    }

    public double getBottomFlywheelDistance() {
        return bottomFlyWheel.getCurrentPosition() / shooterTicksPerRevolution;
    }

    public boolean flyWheelsWithinVelocityTolerance() {
        return topFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, 70.0, Double.POSITIVE_INFINITY)) &&
                bottomFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, 70.0, Double.POSITIVE_INFINITY));
    }

    public void disableFlyWheels() {
        flywheelsEnabled = false;
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

    public LambdaCommand disableFlyWheelPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    disableFlyWheels();
                    stopShooter();
                });
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
                .setIsDone(() -> (
                        topFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, 25.0, Double.POSITIVE_INFINITY)) &&
                        bottomFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, 25.0, Double.POSITIVE_INFINITY))
                        )
                );
    }

    public LambdaCommand setShooterTargetVelocity(double topTarget, double bottomTarget) {
        return new LambdaCommand()
                .setStart(() -> setTargetVelocity(topTarget, bottomTarget))
                .setIsDone(() -> (
                        true
                        /*topFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, 25.0, Double.POSITIVE_INFINITY)) &&
                        bottomFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, 25.0, Double.POSITIVE_INFINITY))*/
                        )
                );
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
