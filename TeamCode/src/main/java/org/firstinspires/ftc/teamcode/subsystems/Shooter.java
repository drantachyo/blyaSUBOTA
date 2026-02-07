package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class Shooter {
    private final DcMotorEx motor;

    public static double TICKS_PER_REV = 28.0; // Для GoBilda, проверь свой мотор!
    public static double MAX_RPM = 6000.0;
    public static double RPM_TOLERANCE = 100;

    // PIDF (Настрой эти цифры в Panels!)
    // F - самое важное. Если не крутится, увеличивай F.
    public static double kF = 14.3;
    public static double kP = 105;
    public static double kI = 0.0;
    public static double kD = 0;
    private double targetRPM = 0;

    public Shooter(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Если дует ВНУТРЬ, убери REVERSE
        motor.setDirection(DcMotor.Direction.REVERSE);

        // Ставим PIDF при старте
        updatePIDF();
    }

    // Вызывай это в loop(), чтобы менять PIDF на лету
    public void update() {
        // Если ты меняешь цифры в Panels, они применятся тут
        PIDFCoefficients currentPIDF = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        if (currentPIDF.p != kP || currentPIDF.f != kF) {
            updatePIDF();
        }
    }

    private void updatePIDF() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
    }

    public void setTargetRPM(double rpm) {
        if(rpm > MAX_RPM) rpm = MAX_RPM;
        this.targetRPM = rpm;

        // ВАЖНО: Сразу отправляем команду мотору!
        double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
        motor.setVelocity(targetTicksPerSec);
    }

    public double getCurrentRPM() {
        return (motor.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public boolean isReady() {
        // Готов, если цель не 0 и мы близко к ней
        return targetRPM > 100 && Math.abs(targetRPM - getCurrentRPM()) < RPM_TOLERANCE;
    }
}