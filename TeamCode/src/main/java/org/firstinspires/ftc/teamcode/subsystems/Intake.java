package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_D;
import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_I;
import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_P;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.config.PIDFConstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Intake implements Subsystem {
    public static PIDCoefficients intakePID = new PIDCoefficients(PIDFConstants.INTAKE_P, PIDFConstants.INTAKE_I, PIDFConstants.INTAKE_D);

    public static final Intake INSTANCE = new Intake();
    private Intake() {};

    private MotorEx intake = new MotorEx(Constants.Intake.intake);

    private ControlSystem intakeControlSystem = ControlSystem.builder()
            .posPid(intakePID)
            .build();

    private boolean intakePIDEnabled = false;
    private double intakeGoalPos = 0.0;

    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    @Override
    public void periodic() {
        if (intakePIDEnabled) {
            intakeControlSystem.setGoal(new KineticState(intakeGoalPos));
            intake.setPower(intakeControlSystem.calculate(new KineticState(intake.getCurrentPosition(), intake.getVelocity())));
        }
        else {
            //intakeControlSystem.setGoal(new KineticState(0.0, 0.0));
        }
    }

    public void enableIntakePID() {
        intakePIDEnabled = true;
    }

    public void disableIntakePID() {
        intakePIDEnabled = false;
    }

    public void intake() {
        intake.setPower(Constants.Intake.intakeSpeed);
    }

    public void outtake() {
        intake.setPower(-Constants.Intake.intakeSpeed);
    }

    public void stop() {
        intake.setPower(0);
    }

    public double getIntakeSpeed() {
        return intake.getPower();
    }

    public double getCurrentPosition() {
        return intake.getCurrentPosition();
    }

    public void resetPosition() {
        intake.atPosition(0.0);
    }

    public void setTargetPosition(double targetPos) {
        intakePIDEnabled = true;
        intakeGoalPos = targetPos;
    }

    public LambdaCommand setIntake() {
        return new LambdaCommand()
                .setStart(() -> intake());
    }

    public LambdaCommand setOuttake() {
        return new LambdaCommand()
                .setStart(() -> outtake());
    }

    public LambdaCommand stopIntake() {
        return new LambdaCommand()
                .setStart(() -> stop());
    }

    public LambdaCommand outtakeWhenFlywheelsReady() {
        return new LambdaCommand()
                .setUpdate(() -> {
                    if (Shooter.INSTANCE.flyWheelsWithinVelocityTolerance()) {
                        intake();
                    }
                    else {
                        stop();
                    }
                });
    }

}
