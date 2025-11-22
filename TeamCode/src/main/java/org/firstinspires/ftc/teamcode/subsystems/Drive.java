package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.Supplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.HolonomicMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();
    private Drive() {};

    double integralSum = 0;
    public static double Kp = 0.01;
    public static double Ki = 0;
    public static double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private boolean headingControl = false;

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
        odo.update();
        if (headingControl) {
            setDrivePowerForPID(PIDControl(Math.toRadians(0), Math.toRadians(Vision.INSTANCE.xCrosshairOffset())));
        }

    }

    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        ActiveOpMode.telemetry().addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public Pose2D getPinpointPosition() {
        return odo.getPosition();
    }

    public MecanumDriverControlled driveControlled(boolean isFieldCentric) {
        if (isFieldCentric) {
            return new MecanumDriverControlled(
                    leftFront,
                    rightFront,
                    leftBack,
                    rightBack,
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX(),
                    new FieldCentric(new Supplier<Angle>() {
                        @Override
                        public Angle get() {
                            return Angle.fromRad(odo.getHeading(AngleUnit.RADIANS));
                        }
                    })
            );
        }
        else {
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

    }

    public GoBildaPinpointDriver getPinpoint() {
        return odo;
    }

    public YawPitchRollAngles getIMUOrientation() {
        return imu.getImu().getRobotYawPitchRollAngles();
    }

    public void zeroPinpoint() {
        odo.resetPosAndIMU();
    }

    public void updatePinpoint() {
        odo.update();
    }

    public double getPinpointHeading() {
        return odo.getHeading(AngleUnit.RADIANS);
    }

    public Command enableHeadingPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingControl = true;
                });
    }

    public Command disableHeadingPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingControl = false;
                });
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

    public Command setDrive(double power) {
        return new LambdaCommand()
                .setStart(() -> {
                    leftFront.setPower(power);
                    leftBack.setPower(power);
                    rightFront.setPower(power);
                    rightBack.setPower(power);
                });
    }

    public void setDrivePowerForPID(double power) {
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(power);
        rightBack.setPower(power);
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
