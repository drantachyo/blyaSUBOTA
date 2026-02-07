package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    private double kP, kI, kD, kF, kS;
    private double lastTime_sec;
    private double period;
    private double lastError;
    private double compoundedI;

    // Лимит интеграла теперь переменная, по умолчанию 0.8 (или сколько тебе нужно)
    private double maxIntegrationVelocity = 0.8;

    public PIDFController(double kP, double kI, double kD, double kF, double kS) {
        setPIDF(kP, kI, kD, kF, kS);
    }

    public void reset() {
        lastError = 0;
        compoundedI = 0;
        lastTime_sec = 0;
    }

    public double calculate(double error) {
        updateTime();

        // P
        double p = error * kP;

        // I (с защитой от windup)
        // Считаем только если период адекватный (меньше 0.1 сек), чтобы при лагах не улетело
        if (period < 0.1) {
            compoundedI += error * period;
            compoundedI = Range.clip(compoundedI, -maxIntegrationVelocity, maxIntegrationVelocity);
        }
        double i = compoundedI * kI;

        // D
        double d = 0;
        if (period > 1E-6) {
            double errorDelta = error - lastError;
            d = (errorDelta / period) * kD;
        }
        lastError = error;

        // FF + Static Friction (kS)
        double ff = kF + (Math.signum(error) * kS);

        return p + i + d + ff;
    }

    private void updateTime() {
        double currentTime_sec = System.nanoTime() / 1E9;
        if (lastTime_sec == 0) {
            period = 0;
        } else {
            period = currentTime_sec - lastTime_sec;
        }
        lastTime_sec = currentTime_sec;
    }

    public void setPIDF(double kP, double kI, double kD, double kF, double kS) {
        this.kP = kP; this.kI = kI; this.kD = kD; this.kF = kF; this.kS = kS;
    }

    // Сеттер для лимита интеграла, если вдруг понадобится тюнить
    public void setMaxIntegrationVelocity(double max) {
        this.maxIntegrationVelocity = max;
    }
}