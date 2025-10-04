package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.DoubleComponent;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.util.controller.implementation.DoubleController;
import dev.frozenmilk.dairy.core.util.supplier.numeric.CachedMotionComponentSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.MotionComponents;
import dev.frozenmilk.mercurial.subsystems.SDKSubsystem;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.util.cell.Cell;
import dev.frozenmilk.util.units.distance.Distance;

public class Shooter extends SDKSubsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() {}

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

    //motors
    private final Cell<DcMotorEx> topFlyWheel = subsystemCell(() -> getHardwareMap().get(DcMotorEx.class, Constants.Shooter.topFlyWheel));
    private final Cell<DcMotorEx> bottomFlyWheel = subsystemCell(() -> getHardwareMap().get(DcMotorEx.class, Constants.Shooter.bottomFlyWheel));
    private final Cell<DcMotorEx> leftPivot = subsystemCell(() -> getHardwareMap().get(DcMotorEx.class, Constants.Shooter.leftPivot));
    private final Cell<DcMotorEx> rightPivot = subsystemCell(() -> getHardwareMap().get(DcMotorEx.class, Constants.Shooter.rightPivot));

    //encoder
    private final Cell<EnhancedDoubleSupplier> encoder = subsystemCell(() -> new EnhancedDoubleSupplier(() -> (double) bottomFlyWheel.get().getCurrentPosition()));
    //current of motor
    private final Cell<EnhancedDoubleSupplier> current = subsystemCell(() -> new EnhancedDoubleSupplier(() -> bottomFlyWheel.get().getCurrent(CurrentUnit.AMPS)));

    //controller
    private double targetPos = 0.0;
    private double targetVel = 0.0;
    private double posTolerance = 50.0;
    private double velTolerance = 50.0;
    private final CachedMotionComponentSupplier<Double> targetSupplier = new CachedMotionComponentSupplier<>(motionComponent -> {
        if (motionComponent == MotionComponents.STATE) {
            return targetPos;
        }
        else if (motionComponent == MotionComponents.VELOCITY) {
            return targetVel;
        }
        return Double.NaN;
    });
    private final CachedMotionComponentSupplier<Double> toleranceSupplier = new CachedMotionComponentSupplier<>(motionComponent -> {
        if (motionComponent == MotionComponents.STATE) {
            return posTolerance;
        }
        else if (motionComponent == MotionComponents.VELOCITY) {
            return velTolerance;
        }
        return Double.NaN;
    });
    private final Cell<DoubleController> controller = subsystemCell(() ->
            new DoubleController(
                    targetSupplier,
                    encoder.get(),
                    toleranceSupplier,
                    (Double power) -> {
                        leftPivot.get().setPower(power);
                        rightPivot.get().setPower(power);
                    },
                    new DoubleComponent.P(MotionComponents.STATE, () -> SHOOTER_P)
                            .plus(new DoubleComponent.I(MotionComponents.STATE, () -> SHOOTER_I))
                            .plus(new DoubleComponent.D(MotionComponents.STATE, () -> SHOOTER_D))
            )
    );

    public double calculateShooterVelocity(double distance) {
        return Math.sqrt((GRAVITY*Math.pow(distance, 2))
                /   2*Math.pow(Math.cos(Math.toRadians(SHOOTER_ANGLE)), 2)*(distance*Math.tan(Math.toRadians(SHOOTER_ANGLE))-(TARGET_HEIGHT- SHOOTER_HEIGHT)));
    }

    public double calculateRPM(double v) {
        //need to experimentally find
        return v;
    }

    //set Target method
    public void setTargetVelocity(double target) {
        controller.get().setEnabled(true);
        this.targetVel = target;
        targetSupplier.reset();
    }


}
