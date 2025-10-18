package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Shooter.*;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
import dev.frozenmilk.mercurial.commands.Lambda;
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
    private final Cell<Servo> launcher = subsystemCell(() -> getHardwareMap().get(Servo.class, Constants.Shooter.launcher));
//    private final Cell<DcMotorEx> leftPivot = subsystemCell(() -> getHardwareMap().get(DcMotorEx.class, Constants.Shooter.leftPivot));
//    private final Cell<DcMotorEx> rightPivot = subsystemCell(() -> getHardwareMap().get(DcMotorEx.class, Constants.Shooter.rightPivot));

    //encoder
    private final Cell<EnhancedDoubleSupplier> bottomEncoder = subsystemCell(() -> new EnhancedDoubleSupplier(() -> (double) bottomFlyWheel.get().getCurrentPosition()));
    private final Cell<EnhancedDoubleSupplier> topEncoder = subsystemCell(() -> new EnhancedDoubleSupplier(() -> (double) topFlyWheel.get().getCurrentPosition()));
    //current of motor
    private final Cell<EnhancedDoubleSupplier> bottomCurrent = subsystemCell(() -> new EnhancedDoubleSupplier(() -> bottomFlyWheel.get().getCurrent(CurrentUnit.AMPS)));

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

    private final Cell<DoubleController> bottomController = subsystemCell(() ->
            new DoubleController(
                    targetSupplier,
                    bottomEncoder.get(),
                    toleranceSupplier,
                    (Double power) -> {
                        bottomFlyWheel.get().setPower(power);
                    },
                    new DoubleComponent.P(MotionComponents.STATE, () -> BOTTOM_SHOOTER_P)
                            .plus(new DoubleComponent.I(MotionComponents.STATE, () -> BOTTOM_SHOOTER_I))
                            .plus(new DoubleComponent.D(MotionComponents.STATE, () -> BOTTOM_SHOOTER_D))
            )
    );

    private final Cell<DoubleController> topController = subsystemCell(() ->
            new DoubleController(
                    targetSupplier,
                    topEncoder.get(),
                    toleranceSupplier,
                    (Double power) -> {
                        topFlyWheel.get().setPower(power);
                    },
                    new DoubleComponent.P(MotionComponents.STATE, () -> TOP_SHOOTER_P)
                            .plus(new DoubleComponent.I(MotionComponents.STATE, () -> TOP_SHOOTER_I))
                            .plus(new DoubleComponent.D(MotionComponents.STATE, () -> TOP_SHOOTER_D))
            )
    );

    public double calculateShooterVelocity(double distance) {
        return Math.sqrt((GRAVITY*Math.pow(distance, 2))
                /   2*Math.pow(Math.cos(Math.toRadians(SHOOTER_ANGLE)), 2)*(distance*Math.tan(Math.toRadians(SHOOTER_ANGLE))-(TARGET_HEIGHT-SHOOTER_HEIGHT)));
    }

    public double calculateRPM(double v) {
        //need to experimentally find
        return v;
    }

    //set Target method
    public void setTargetVelocity(double target) {
        topController.get().setEnabled(true);
        bottomController.get().setEnabled(true);
        this.targetVel = target;
        targetSupplier.reset();
    }

    public void setLauncher() {
        launcher.get().setPosition(LAUNCH_POS);
    }

    public void resetLauncher() {
        launcher.get().setPosition(FLAT_POS);
    }

    public void launcherPosition(double pos) {
        launcher.get().setPosition(pos);
    }

    public void intake() {
        topController.get().setEnabled(false);
        bottomController.get().setEnabled(false);
        topFlyWheel.get().setPower(0.6);
        bottomFlyWheel.get().setPower(0.6);
    }

    public void outtake() {
        topController.get().setEnabled(false);
        bottomController.get().setEnabled(false);
        topFlyWheel.get().setPower(-0.6);
        bottomFlyWheel.get().setPower(-0.6);
    }

    public void stop() {
        topFlyWheel.get().setPower(0);
        bottomFlyWheel.get().setPower(0);
    }

    public Lambda setIntake() {
        return new Lambda("setIntake")
                .setInit(() -> intake());
    }

    public Lambda setOuttake() {
        return new Lambda("setOuttake")
                .setInit(() -> outtake());
    }

    public Lambda stopShooter() {
        return new Lambda("stopIntake")
                .setInit(() -> stop());
    }

    public Lambda setShooterTargetVelocity(double target) {
        return new Lambda("setShooterTargetVelocity")
                .setInit(() -> setShooterTargetVelocity(target));
    }

    public Lambda launchBall() {
        return new Lambda("launchBall")
                .setInit(() -> setLauncher());
    }

    public Lambda resetBallLauncher() {
        return new Lambda("launchBall")
                .setInit(() -> resetLauncher());
    }

    public Lambda setLauncherPosition(double pos) {
        return new Lambda("setLauncherPosition")
                .setInit(() -> launcherPosition(pos));
    }

}
