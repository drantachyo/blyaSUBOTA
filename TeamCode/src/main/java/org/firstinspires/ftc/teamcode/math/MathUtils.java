package org.firstinspires.ftc.teamcode.math;

public class MathUtils {

    // Приводит угол к диапазону от -PI до +PI
    // (Используется для PID контроллеров поворота)
    public static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}