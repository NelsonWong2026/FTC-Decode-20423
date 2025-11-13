package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
/*
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
*/
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.util.cell.Cell;

public class Vision extends SDKSubsystem {
    public static final Vision INSTANCE = new Vision();
    private Vision() {}

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach{}

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    private final Cell<Limelight3A> limelight = subsystemCell(() -> getHardwareMap().get(Limelight3A.class, "limelight"));
    // Retrieve the IMU from the hardware map
    final Cell<IMU> imu = subsystemCell(() -> getHardwareMap().get(IMU.class, "imu"));
    // Adjust the orientation parameters to match your robot
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    public void startLimelight() {
        limelight.get().start();
    }

    public void stopLimelight() {
        limelight.get().stop();
    }

    public Pose3D getBotPose() {
        LLResult result = limelight.get().getLatestResult();
        YawPitchRollAngles orientation = imu.get().getRobotYawPitchRollAngles();
        limelight.get().updateRobotOrientation(orientation.getYaw());
        if (result != null && result.isValid()) {
            return result.getBotpose_MT2();
        }
        else {
            return null;
        }
    }

    public LLResult getLLResult() {
        LLResult result = limelight.get().getLatestResult();
        if (result != null && result.isValid()) {
            return result;
        }
        else {
            return null;
        }
    }

    public double DistanceFromGoal() {
        LLResult result = limelight.get().getLatestResult();
        double distance;
        if (result != null && result.isValid()) {
            return (Constants.Vision.aprilTagHeight-Constants.Vision.llHeight)/Math.tan(-result.getTy()+Constants.Vision.llAngleAboveGround);
        }
        return 0;
    }


}
