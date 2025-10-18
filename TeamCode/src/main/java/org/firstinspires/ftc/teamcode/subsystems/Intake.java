package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_D;
import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_I;
import static org.firstinspires.ftc.teamcode.Constants.Intake.INTAKE_P;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.DoubleComponent;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
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

public class Intake extends SDKSubsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() {}

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
    private final Cell<CachingDcMotorEx> intake = subsystemCell(() -> new CachingDcMotorEx(getHardwareMap().get(DcMotorEx.class, Constants.Intake.intake)));

    //encoder
    private final Cell<EnhancedDoubleSupplier> encoder = subsystemCell(() -> new EnhancedDoubleSupplier(() -> (double) intake.get().getCurrentPosition()));
    //current of motor
    private final Cell<EnhancedDoubleSupplier> current = subsystemCell(() -> new EnhancedDoubleSupplier(() -> intake.get().getCurrent(CurrentUnit.AMPS)));

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
                        intake.get().setPower(power);
                    },
                    new DoubleComponent.P(MotionComponents.STATE, () -> INTAKE_P)
                            .plus(new DoubleComponent.I(MotionComponents.STATE, () -> INTAKE_I))
                            .plus(new DoubleComponent.D(MotionComponents.STATE, () -> INTAKE_D))
            )
    );

    public void intake() {
        controller.get().setEnabled(false);
        intake.get().setPower(Constants.Intake.intakeSpeed);
    }

    public void outtake() {
        controller.get().setEnabled(false);
        intake.get().setPower(-Constants.Intake.intakeSpeed);
    }

    public void stop() {
        intake.get().setPower(0);
    }

    public Lambda setIntake() {
        return new Lambda("setIntake")
                .setInit(() -> intake());
    }

    public Lambda setOuttake() {
        return new Lambda("setOuttake")
                .setInit(() -> outtake());
    }

    public Lambda stopIntake() {
        return new Lambda("stopIntake")
                .setInit(() -> stop());
    }





}
