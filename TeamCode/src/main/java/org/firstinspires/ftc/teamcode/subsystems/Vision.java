package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

public class Vision implements Subsystem {
    public static final Vision INSTANCE = new Vision();
    private Vision() {};

    private Limelight3A limelight;
    private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.LEFT);
    private GoBildaPinpointDriver odo;

    @Override
    public void initialize() {
        // initialization logic (runs on init)
        odo = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class, Constants.Drive.pinpoint);
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, Constants.Vision.limelight);
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        odo.update();
    }

    public void startLimelight() {
        limelight.start();
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public Pose3D getBotPose() {
        LLResult result = limelight.getLatestResult();
        YawPitchRollAngles orientation = imu.getImu().getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        else {
            return null;
        }
    }

    public LLResult getLLResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result;
        }
        else {
            return null;
        }
    }

    public double DistanceFromGoal() {
        LLResult result = limelight.getLatestResult();
        double distance;
        if (result != null && result.isValid()) {
            return (Constants.Vision.aprilTagHeight-Constants.Vision.llHeight)/Math.tan(-result.getTy()+Constants.Vision.llAngleAboveGround);
        }
        return 0;
    }

    public void updateOrientationWithIMU() {
        YawPitchRollAngles orientation = imu.getImu().getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
    }


}
