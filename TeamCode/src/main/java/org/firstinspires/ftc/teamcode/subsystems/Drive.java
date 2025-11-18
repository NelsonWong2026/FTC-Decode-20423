package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.HolonomicMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    private Drive() {};

    // put hardware, commands, etc here
    private MotorEx leftFront = new MotorEx(Constants.Drive.leftFront).brakeMode().reversed();
    private MotorEx leftBack = new MotorEx(Constants.Drive.leftBack).brakeMode().reversed();
    private MotorEx rightFront = new MotorEx(Constants.Drive.rightFront).brakeMode();
    private MotorEx rightBack = new MotorEx(Constants.Drive.rightBack).brakeMode();
    private GoBildaPinpointDriver odo;
    private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.LEFT).zeroed();
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    @Override
    public void initialize() {
        // initialization logic (runs on init)
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, Constants.Drive.pinpoint);
        odo.setOffsets(Constants.Drive.odomXOffset, Constants.Drive.odomYOffset, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }

    public Pose2D getPinpointPosition() {
        return odo.getPosition();
    }

    public Command driveControlled() {
        return new MecanumDriverControlled(
                leftFront,
                rightFront,
                leftBack,
                rightBack,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
    }

    public Command driveForward() {
        return new LambdaCommand()
                .setStart(() -> {
                    leftFront.setPower(1);
                    leftBack.setPower(1);
                    rightFront.setPower(1);
                    rightBack.setPower(1);
                });
    }

    public Command stopDrive() {
        return new LambdaCommand()
                .setStart(() -> {
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                });
    }

    public Command driveCommand(boolean isFieldCentric) {
        GamepadEx gamepad1 = Gamepads.gamepad1();
        return new LambdaCommand()
                .setRequirements(this)
                .setUpdate(() -> {
                    double y = gamepad1.leftStickY().negate().get();
                    double x = gamepad1.leftStickX().get();
                    double rx = gamepad1.rightStickX().get();

                    if (gamepad1.a().toggleOnBecomesTrue().get()) {
                        imu.zeroed();
                    }

                    double botHeading = imu.get().inRad;

                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX *= 1.1;

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;

                    double denominator2 = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower2 = (y + x + rx) / denominator;
                    double backLeftPower2 = (y - x + rx) / denominator;
                    double frontRightPower2 = (y - x - rx) / denominator;
                    double backRightPower2 = (y + x - rx) / denominator;

                    if (isFieldCentric && gamepad1.rightBumper().get()) {
                        leftFront.setPower(frontLeftPower / 4);
                        leftBack.setPower(backLeftPower / 4);
                        rightFront.setPower(frontRightPower / 4);
                        rightBack.setPower(backRightPower / 4);
                    }
                    else if (isFieldCentric) {
                        leftFront.setPower(frontLeftPower);
                        leftBack.setPower(backLeftPower);
                        rightFront.setPower(frontRightPower);
                        rightBack.setPower(backRightPower);
                    }
                    else if (gamepad1.rightBumper().get()) {
                        leftFront.setPower(frontLeftPower2);
                        leftBack.setPower(backLeftPower2);
                        rightFront.setPower(frontRightPower2);
                        rightBack.setPower(backRightPower2);
                    }
                    else {
                        leftFront.setPower(frontLeftPower2);
                        leftBack.setPower(backLeftPower2);
                        rightFront.setPower(frontRightPower2);
                        rightBack.setPower(backRightPower2);
                    }
                    /*leftFront.setPower(y);
                    leftBack.setPower(y);
                    rightFront.setPower(-gamepad1.rightStickY().state());
                    rightBack.setPower(-gamepad1.rightStickY().state());*/
                })
                .setIsDone(() -> false);
    }

    public LambdaCommand tankDriveCommand() {
        GamepadEx gamepad1 = Gamepads.gamepad1();
        return new LambdaCommand()
                .setRequirements(this)
                .setUpdate(() -> {
                    if (gamepad1.rightBumper().get()) {
                        leftFront.setPower(gamepad1.leftStickY().get() / 4);
                        leftBack.setPower(gamepad1.leftStickY().get() / 4);
                        rightFront.setPower(gamepad1.rightStickY().get() / 4);
                        rightBack.setPower(gamepad1.rightStickY().get() / 4);
                    }
                    else {
                        leftFront.setPower(gamepad1.leftStickY().get());
                        leftBack.setPower(gamepad1.leftStickY().get());
                        rightFront.setPower(gamepad1.rightStickY().get());
                        rightBack.setPower(gamepad1.rightStickY().get());
                    }
                });
    }
}
