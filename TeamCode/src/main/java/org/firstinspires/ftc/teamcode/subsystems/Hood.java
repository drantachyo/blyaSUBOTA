package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Hood {
    private final Servo servo;

    public static double LIMIT_BOTTOM = 1.0; // Парковка (внизу)
    public static double LIMIT_TOP = 0.0;    // Макс подъем (вверху)

    // Настройки компенсации (поднимаем худ, если обороты падают)
    public static double COMP_FACTOR = 0.002;
    public static double RPM_THRESHOLD = 200;

    private double basePos = 0.15; // Дефолт

    public Hood(HardwareMap hw) {
        servo = hw.get(Servo.class, "hood");
    }

    public void setBasePosition(double pos) {
        // Защита от дурака: клипаем между мин и макс
        this.basePos = Range.clip(pos, Math.min(LIMIT_TOP, LIMIT_BOTTOM), Math.max(LIMIT_TOP, LIMIT_BOTTOM));
    }

    public void update(double currentRPM, double targetRPM) {
        if (targetRPM < 100) {
            servo.setPosition(basePos);
            return;
        }

        double error = targetRPM - currentRPM;
        // Игнорируем мелкий шум (RPM_THRESHOLD) и перекрут (error < 0)
        if (Math.abs(error) < RPM_THRESHOLD || error < 0) error = 0;

        // Компенсация: чем больше просадка оборотов, тем выше задираем нос (уменьшаем значение, т.к. TOP=0)
        double targetPos = basePos - (error * COMP_FACTOR);

        servo.setPosition(Range.clip(targetPos, Math.min(LIMIT_TOP, LIMIT_BOTTOM), Math.max(LIMIT_TOP, LIMIT_BOTTOM)));
    }

    // === ВОТ ГЕТТЕР, КОТОРЫЙ ТЫ ПРОСИЛ ===
    public double getServoPosition() {
        return servo.getPosition();
    }
}