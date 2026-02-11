package org.firstinspires.ftc.teamcode.math;

public class MathUtils {
    // Приводит любой угол (хоть 720, хоть -1000) в диапазон -180...+180
    public static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}