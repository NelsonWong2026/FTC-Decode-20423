package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Vision implements Subsystem {
    public static final Vision INSTANCE = new Vision();
    private Vision() {};

    private Limelight3A limelight;

    @Override
    public void initialize() {
        // initialization logic (runs on init)
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, Constants.Vision.limelight);
    }

    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        updateOrientationWithPinpoint();
    }

    public void startLimelight() {
        limelight.start();
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public Pose3D getBotPose() {
        LLResult result = limelight.getLatestResult();
        limelight.updateRobotOrientation(Drive.INSTANCE.getPinpointHeading());
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

    public double xCrosshairOffset() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0.0;
    }

    public double DistanceFromGoal() {
        LLResult result = limelight.getLatestResult();
        double distance;
        if (result != null && result.isValid()) {
            return (Constants.Vision.aprilTagHeight -Constants.Vision.llHeight)/Math.tan(-result.getTy()+Constants.Vision.llAngleAboveGround);
        }
        return 0;
    }

    public void updateOrientationWithIMU() {
        limelight.updateRobotOrientation(Drive.INSTANCE.getIMUOrientation().getYaw());
    }

    public void updateOrientationWithPinpoint() {
        limelight.updateRobotOrientation(Drive.INSTANCE.getPinpointHeading());
    }


}
