package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
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
        limelight.updateRobotOrientation(Drive.INSTANCE.getPinpointHeadingRad());
        if (result != null && result.isValid()) {
            return result.getBotpose();
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
        if (result != null && result.isValid() &&
                (result.getFiducialResults().get(0).getFiducialId() == 20 ||
                        result.getFiducialResults().get(0).getFiducialId() == 24)) {
            return result.getTx();
        }
        return 0.0;
    }

    public double angleToFaceGoalLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result.getFiducialResults().get(0).getFiducialId() == 24) {
                return 90 - Math.atan(Math.abs(Constants.Shooter.RED_GOAL_POSE.getY() - (result.getBotpose().getPosition().y*Constants.Drive.meterToCoordinateConversion)) /
                        Math.abs(Constants.Shooter.RED_GOAL_POSE.getX() - result.getBotpose().getPosition().x*Constants.Drive.meterToCoordinateConversion)) * 180 / Math.PI;
            }
            else if (result.getFiducialResults().get(0).getFiducialId() == 20) {
                return 90 - Math.atan((Constants.Shooter.BLUE_GOAL_POSE.getY() - result.getBotpose().getPosition().y*Constants.Drive.meterToCoordinateConversion) /
                        (Constants.Shooter.BLUE_GOAL_POSE.getX() - result.getBotpose().getPosition().x*Constants.Drive.meterToCoordinateConversion)) * 180/Math.PI;
            }
        }
        return 0.0;
    }

    public boolean aprilTagAreaGreaterThan(double area) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotposeAvgArea() > area;
        }
        return false;
    }

    /*public double angleToFaceGoalOdometry() {
        return  90 - Math.atan(
                Math.abs(Constants.Shooter.RED_GOAL_POSE.getY() - (Drive.INSTANCE.getPinpointPosition().getY(DistanceUnit.METER)*Constants.Drive.meterToCoordinateConversion)) /
                Math.abs(Constants.Shooter.RED_GOAL_POSE.getX() - (Drive.INSTANCE.getPinpointPosition().getX(DistanceUnit.METER)*Constants.Drive.meterToCoordinateConversion))
        ) * 180 / Math.PI;
    }*/

    public void relocalizeWithLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && aprilTagAreaGreaterThan(0.4)) {
            Pose3D botPose = result.getBotpose();
            Drive.INSTANCE.setPinpointPosition(new Pose2D(DistanceUnit.METER, botPose.getPosition().x, botPose.getPosition().y,
                    AngleUnit.RADIANS, Drive.INSTANCE.getPinpointHeadingRad()));
            ActiveOpMode.gamepad1().rumble(200);
        }
    }

    public Command relocalizeWithLimelightCommand() {
        return new LambdaCommand()
                .setStart(() -> relocalizeWithLimelight());
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
        limelight.updateRobotOrientation(Drive.INSTANCE.getPinpointHeadingRad());
    }


}
