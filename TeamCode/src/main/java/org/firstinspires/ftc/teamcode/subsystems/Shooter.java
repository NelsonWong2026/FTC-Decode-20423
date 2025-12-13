package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.*;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.BOTTOM_SHOOTER_D;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.BOTTOM_SHOOTER_Ks;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.BOTTOM_SHOOTER_Kv;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.BOTTOM_SHOOTER_I;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.BOTTOM_SHOOTER_P;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.TOP_SHOOTER_D;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.TOP_SHOOTER_Ks;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.TOP_SHOOTER_Kv;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.TOP_SHOOTER_I;
import static org.firstinspires.ftc.teamcode.config.PIDFConstants.TOP_SHOOTER_P;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
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

    public static BasicFeedforwardParameters topFlyWheelFeedforward = new BasicFeedforwardParameters(TOP_SHOOTER_Kv, 0, TOP_SHOOTER_Ks);
    public static BasicFeedforwardParameters bottomFlyWheelFeedforward = new BasicFeedforwardParameters(BOTTOM_SHOOTER_Kv, 0, BOTTOM_SHOOTER_Ks);

    public static final Shooter INSTANCE = new Shooter();
    private Shooter() {};

    private MotorEx topFlyWheel = new MotorEx(Constants.Shooter.topFlyWheel).floatMode();
    private MotorEx bottomFlyWheel = new MotorEx(Constants.Shooter.bottomFlyWheel).floatMode().reversed();
    private ServoEx blocker = new ServoEx(Constants.Shooter.blocker);

    private final TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
    private double targetVel = 0.0;
    private double topTargetVel = 0.0;
    private double bottomTargetVel = 0.0;
    private double intakeGoal = 180.0;
    private double intakePosition = 0.0;
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

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        OPEN_GATE,
        LAUNCH,
        RESET_GATE
    }

    private FlywheelState flywheelState;

    private double GATE_OPEN_AND_CLOSE_TIME = 1.5;
    private double SHOOT_TIME = 0.15;
    private double WAIT_TIME = 0.7;
    private int shotsRemaining = 0;
    private double FLYWHEEL_MAX_SPINUP_TIME = 4;

    @Override
    public void initialize() {
        // initialization logic (runs on init)
        /*private DcMotorEx topWheel = ActiveOpMode.hardwareMap().get(DcMotorEx.class, Constants.Shooter.topFlyWheel);
        private DcMotorEx bottomWheel = ActiveOpMode.hardwareMap().get(DcMotorEx.class, Constants.Shooter.bottomFlyWheel);
        topWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/
        flywheelState = FlywheelState.IDLE;
        setBlocker();
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

    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    intakePosition = Intake.INSTANCE.getCurrentPosition();
                    clearBlocker();
                    stateTimer.reset();
                    flywheelState = FlywheelState.OPEN_GATE;
                }
                break;
            case OPEN_GATE:
                if (stateTimer.seconds() > GATE_OPEN_AND_CLOSE_TIME) {
                    stateTimer.reset();
                    Intake.INSTANCE.setTargetPosition(intakePosition + intakeGoal);
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > SHOOT_TIME) {
                    Intake.INSTANCE.stop();
                    Intake.INSTANCE.disableIntakePID();
                    intakeGoal += 180;
                    shotsRemaining--;
                    stateTimer.reset();

                    flywheelState = FlywheelState.RESET_GATE;
                }
                break;
            case RESET_GATE:
                if (shotsRemaining > 0 && (stateTimer.seconds() > WAIT_TIME /*|| flyWheelsWithinVelocityTolerance(15)*/)) {
                    stateTimer.reset();
                    Intake.INSTANCE.setTargetPosition(intakePosition + intakeGoal);
                    flywheelState = FlywheelState.LAUNCH;
                }
                else if (shotsRemaining == 0 && stateTimer.seconds() > 0.3){
                    intakeGoal = 180;
                    Intake.INSTANCE.disableIntakePID();
                    setBlocker();
                    flywheelState = FlywheelState.IDLE;
                }
                break;
        }
    }

    public FlywheelState getFlyWheelState() {
        return flywheelState;
    }

    public double getShotsRemaining() {
        return shotsRemaining;
    }

    public void shootOnTime(double time) {
        ElapsedTime actionTimer = new ElapsedTime();
        actionTimer.reset();
        Intake.INSTANCE.setIntake();
        if (actionTimer.seconds() > time) {
            Intake.INSTANCE.stopIntake();
        }
    }

    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
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
        return topFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, veloTolerance, Double.POSITIVE_INFINITY)) &&
                bottomFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, veloTolerance, Double.POSITIVE_INFINITY));
    }

    public boolean flyWheelsWithinVelocityTolerance(double tolerance) {
        return topFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, tolerance, Double.POSITIVE_INFINITY)) &&
                bottomFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, tolerance, Double.POSITIVE_INFINITY));
    }

    public void disableFlyWheels() {
        flywheelsEnabled = false;
    }

    public void enableFlyWheels() {flywheelsEnabled = true;}

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
                        topFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, veloTolerance, Double.POSITIVE_INFINITY)) &&
                        bottomFlywheelControlSystem.isWithinTolerance(new KineticState(Double.POSITIVE_INFINITY, veloTolerance, Double.POSITIVE_INFINITY))
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
