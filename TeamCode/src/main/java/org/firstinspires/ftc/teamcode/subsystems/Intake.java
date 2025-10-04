package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotor;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
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
    private final Cell<CachingDcMotor> intake = subsystemCell(() -> new CachingDcMotor(getHardwareMap().get(DcMotor.class, Constants.Intake.intake)));

    public void intake() {
        intake.get().setPower(Constants.Intake.intakeSpeed);
    }

    public void outtake() {
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
