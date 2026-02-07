package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.Range;

public class ShooterMath {

    // === ГРАНИЦЫ ТВОЕЙ ТАБЛИЦЫ ===
    // Впиши сюда минимальный и максимальный X (дистанцию),
    // на которых ты проводил замеры.
    // Если дистанция будет меньше 24, мы будем считать её как 24.
    // Если больше 140, будем считать как 140.
    public static double MIN_DIST = 34.9;  // <-- ПОМЕНЯЙ НА СВОЙ МИНИМУМ
    public static double MAX_DIST = 81; // <-- ПОМЕНЯЙ НА СВОЙ МАКСИМУМ

    /**
     * Считает RPM.
     * Формула: y = 0.00672846x^3 - 0.922392x^2 + 56.63474x + 2029.71146
     */
    public static double calculateRPM(double distance) {
        // 1. Сначала обрезаем дистанцию, чтобы не считать "погоду" за пределами графика
        double x = Range.clip(distance, MIN_DIST, MAX_DIST);

        double y = 0.00672846 * Math.pow(x, 3)
                - 0.922392   * Math.pow(x, 2)
                + 56.63474   * x
                + 2050.71146;

        return Range.clip(y, 0, 4800);
    }

    /**
     * Считает Hood.
     * Формула: y = -0.0000148015x^3 + 0.00278868x^2 - 0.177262x + 3.91881
     */
    public static double calculateHood(double distance) {
        // 1. Тоже обрезаем дистанцию
        double x = Range.clip(distance, MIN_DIST, MAX_DIST);

        double y = -0.0000148015 * Math.pow(x, 3)
                + 0.00278868    * Math.pow(x, 2)
                - 0.177262      * x
                + 3.91881;

        return Range.clip(y, 0.0, 1.0);
    }
}