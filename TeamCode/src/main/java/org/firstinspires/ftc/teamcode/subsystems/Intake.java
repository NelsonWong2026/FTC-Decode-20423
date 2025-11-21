package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_D;
import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_I;
import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_P;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() {};

    private MotorEx intake = new MotorEx(Constants.Intake.intake);

    public void intake() {
        intake.setPower(Constants.Intake.intakeSpeed);
    }

    public void outtake() {
        intake.setPower(-Constants.Intake.intakeSpeed);
    }

    public void stop() {
        intake.setPower(0);
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



    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }

}
