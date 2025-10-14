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
/*
    private final Cell<Limelight3A> limelight = subsystemCell(() -> getHardwareMap().get(Limelight3A.class, "limelight"));

    public void startLimelight() {
        limelight.get().start();
    }

    public void stopLimelight() {
        limelight.get().stop();
    }

    public Pose2D getBotPose() {
        LLResult result = limelight.get().getLatestResult();
        Position position = result.getBotpose_MT2().getPosition();
        YawPitchRollAngles orientation = result.getBotpose_MT2().getOrientation();
        return new Pose2D(position.unit, position.x, position.y, AngleUnit.RADIANS, orientation.getYaw());
    }
*/

}
