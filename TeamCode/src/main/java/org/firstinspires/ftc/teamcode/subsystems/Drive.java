package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.config.PIDFConstants;

import java.util.function.Supplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class Drive implements Subsystem {
    public static PIDCoefficients headingPID = new PIDCoefficients(
            PIDFConstants.HEADING_P, PIDFConstants.HEADING_I, PIDFConstants.HEADING_D
    );

    public static final Drive INSTANCE = new Drive();
    private Drive() {};

    private int headingControl = -1;
    private double headingGoal = 0.0;
    private double headingOffsetLimelight = 0.0;

    // put hardware, commands, etc here
    private MotorEx leftFront = new MotorEx(Constants.Drive.leftFront).brakeMode().reversed();
    private MotorEx leftBack = new MotorEx(Constants.Drive.leftBack).brakeMode().reversed();
    private MotorEx rightFront = new MotorEx(Constants.Drive.rightFront).brakeMode();
    private MotorEx rightBack = new MotorEx(Constants.Drive.rightBack).brakeMode();
    private GoBildaPinpointDriver odo;
    private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.LEFT).zeroed();
    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    private ControlSystem headingControlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES,
                    feedback -> feedback.posPid(headingPID))
            .build();

    @Override
    public void initialize() {
        // initialization logic (runs on init)
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, Constants.Drive.pinpoint);
        odo.setOffsets(Constants.Drive.odomYOffset, Constants.Drive.odomXOffset, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        odo.update();
        if (headingControl == 0) {
            headingControlSystem.setGoal(new KineticState(headingGoal));
            setDrivePowerForPID(headingControlSystem.calculate(
                    new KineticState(Vision.INSTANCE.xCrosshairOffset(),
                            odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)))
            );
        }
        else if (headingControl == 1) {
            headingControlSystem.setGoal(new KineticState(headingGoal));
            setDrivePowerForPID(headingControlSystem.calculate(
                    new KineticState(odo.getHeading(AngleUnit.DEGREES),
                            odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)))
            );
        }
        else if (headingControl == 2) {
            headingControlSystem.setGoal(new KineticState(headingGoal));
            setDrivePowerForPID(headingControlSystem.calculate(
                    new KineticState(odo.getHeading(AngleUnit.DEGREES),
                            odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)))
            );
        }
        Pose3D botPose = Vision.INSTANCE.getBotPose();
        /*try {
            if (Vision.INSTANCE.aprilTagAreaGreaterThan(0.4)) {
                odo.setPosition(new Pose2D(DistanceUnit.METER, botPose.getPosition().x, botPose.getPosition().y, AngleUnit.RADIANS, odo.getHeading(AngleUnit.RADIANS)));
            }
        } catch (Exception e) {}*/
    }

    public Pose2D getPinpointPosition() {
        return odo.getPosition();
    }

    public double getHeadingVelocity() {
        return odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    public void setHeadingGoal(double headingGoal) {
        this.headingGoal = headingGoal;
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

    public MecanumDriverControlled redDriveControlled() {
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
                        return Angle.fromRad(odo.getHeading(AngleUnit.RADIANS)+Math.PI/2);
                    }
                })
        );
    }
    public MecanumDriverControlled blueDriveControlled() {
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
                        return Angle.fromRad(odo.getHeading(AngleUnit.RADIANS)+3*Math.PI/2);
                    }
                })
        );
    }

    public MecanumDriverControlled testDriveControlled() {
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
                        return Angle.fromRad(odo.getHeading(AngleUnit.RADIANS)+Math.PI);
                    }
                })
        );
    }

    public MecanumDriverControlled driveControlledIMU(boolean isFieldCentric) {
        if (isFieldCentric) {
            return new MecanumDriverControlled(
                    leftFront,
                    rightFront,
                    leftBack,
                    rightBack,
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX(),
                    new FieldCentric(imu)
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

    public double angleToFaceRedGoalOdometry() {
        return  180 + Math.toDegrees(Math.atan2(
                (Constants.Shooter.RED_GOAL_POSE.getY() - (Drive.INSTANCE.getPinpointPosition().getY(DistanceUnit.METER)*Constants.Drive.meterToCoordinateConversion)),
                (Constants.Shooter.RED_GOAL_POSE.getX() - (Drive.INSTANCE.getPinpointPosition().getX(DistanceUnit.METER)*Constants.Drive.meterToCoordinateConversion))));
    }
    public double angleToFaceBlueGoalOdometry() {
        return  180 + Math.toDegrees(Math.atan2(
                (Constants.Shooter.BLUE_GOAL_POSE.getY() - (Drive.INSTANCE.getPinpointPosition().getY(DistanceUnit.METER)*Constants.Drive.meterToCoordinateConversion)),
                (Constants.Shooter.BLUE_GOAL_POSE.getX() - (Drive.INSTANCE.getPinpointPosition().getX(DistanceUnit.METER)*Constants.Drive.meterToCoordinateConversion))));
    }

    public void setHeadingto90() {
        odo.setHeading(Math.PI/2, AngleUnit.RADIANS);
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

    public Command setPinpointPos(Pose2D pose) {
        return new LambdaCommand()
                .setStart(() -> odo.setPosition(new Pose2D(DistanceUnit.METER, pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER),
                        AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS))));
    }

    public void setPinpointPosition(Pose2D pose) {
        odo.setPosition(new Pose2D(DistanceUnit.METER, pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER),
                AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS)));
    }

    public void zeroIMU() {
        imu.zeroed();
    }

    public void updatePinpoint() {
        odo.update();
    }

    public double getPinpointHeadingRad() {
        return odo.getHeading(AngleUnit.RADIANS);
    }

    public double getPinpointHeadingDeg() {
        return odo.getHeading(AngleUnit.DEGREES);
    }

    public Command enableLimelightHeadingPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingGoal = 0;
                    headingControl = 0;
                    if (Vision.INSTANCE.xCrosshairOffset() != 0.0) {
                        ActiveOpMode.gamepad1().rumble(200);
                    }
                });
    }

    public Command enableRedLimelightHeadingStopPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingGoal = odo.getHeading(AngleUnit.DEGREES) + Vision.INSTANCE.xCrosshairOffset();
                    headingControl = 2;
                    if (Vision.INSTANCE.xCrosshairOffset() != 0.0) {
                        ActiveOpMode.gamepad1().rumble(200);
                    }
                });
    }
    public Command enableBlueLimelightHeadingStopPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingGoal = odo.getHeading(AngleUnit.DEGREES) - Vision.INSTANCE.xCrosshairOffset();
                    headingControl = 2;
                    if (Vision.INSTANCE.xCrosshairOffset() != 0.0) {
                        ActiveOpMode.gamepad1().rumble(200);
                    }
                });
    }

    public Command enableRedHeadingPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingGoal = angleToFaceRedGoalOdometry();
                    headingControl = 1;
                });
    }

    public Command enableRedHeadingLimelight() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingGoal = Vision.INSTANCE.angleToFaceGoalLimelight();
                    headingControl = 1;
                });
    }

    public Command enableBlueHeadingPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingGoal = angleToFaceBlueGoalOdometry();
                    headingControl = 1;
                });
    }

    public Command enableZeroHeadingPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingGoal = 0;
                    headingControl = 1;
                });
    }

    public Command enable180HeadingPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingGoal = 180;
                    headingControl = 1;
                });
    }

    public Command disableHeadingPID() {
        return new LambdaCommand()
                .setStart(() -> {
                    headingControl = -1;
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

    public Command setDriveTurn(double power) {
        return new LambdaCommand()
                .setStart(() -> {
                    leftFront.setPower(-power);
                    leftBack.setPower(-power);
                    rightFront.setPower(power);
                    rightBack.setPower(power);
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
